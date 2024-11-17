use defmt::{debug, error, info, warn};
use embassy_net::tcp::TcpSocket;
use httparse::Header;

use crate::server::webdav::error::ToStaticString as _;

use super::error::Error;

pub(super) fn find_header<'a, 'b>(
    headers: &'a [Header],
    header_name: &'b str,
) -> impl Iterator<Item = &'a Header<'a>> + 'b
where
    'a: 'b,
{
    headers
        .iter()
        .filter(move |header| header_name.eq_ignore_ascii_case(header.name))
}

fn parse<'a, 'b>(
    buf: &'a [u8; 4096],
    n: usize,
    headers: &'b mut [Header<'b>; 16],
) -> Result<(usize, usize), Error>
where
    'a: 'b,
    'b: 'a,
{
    let mut req = httparse::Request::new(headers);
    match req.parse(&buf[..n]) {
        Ok(httparse::Status::Partial) => {
            error!("Partial response: {} {}", req.method, req.path);
            for i in 0..req.headers.len() {
                error!(
                    "header {}: {} -> {}",
                    i,
                    req.headers[i].name,
                    core::str::from_utf8(req.headers[i].value).unwrap()
                );
            }
            Err(Error::IncompleteHeaderRead)
        }
        Err(e) => {
            error!("Header parsing failed: {}", e.to_string());
            Err(Error::HttpParse(e))
        }
        Ok(httparse::Status::Complete(p)) => {
            info!("Complete response: {} {}", req.method, req.path);
            for i in 0..req.headers.len() {
                debug!(
                    "header {}: {} -> {}",
                    i,
                    req.headers[i].name,
                    core::str::from_utf8(req.headers[i].value).unwrap()
                );
            }
            debug!("Body starts at: {}", p);
            info!(
                "Body: (len: {}) starts: {}",
                &buf[p..n].len(),
                core::str::from_utf8(&buf[p..(p + 5)]).unwrap()
            );

            for content_length in find_header(headers, "CONTENT-LENGTH") {
                debug!("56: content-length: {}", content_length.value);
                let value = core::str::from_utf8(content_length.value)?.parse::<u64>()?;
                debug!("    value: {}", value);
            }

            let mut content_lengths = find_header(headers, "CONTENT-LENGTH");
            let content_length = if let Some(content_length_header) = content_lengths.next() {
                core::str::from_utf8(content_length_header.value)?.parse::<usize>()?
            } else {
                return Err(Error::UnknownContentLength);
            };
            if content_lengths.next().is_none() {
                Ok((p, content_length))
            } else {
                Err(Error::MultipleContentLengths)
            }
        }
    }
}

fn parse_request_line<'a>(buf: &'a [u8; 4096]) -> Result<(&'a [u8], &'a [u8]), Error> {
    let mut pos = 0;
    while (buf[pos] == b'\n' || buf[pos] == b'\r') {
        pos += 1;
    }
    let method_start = pos;
    while (buf[pos] != b' ') {
        pos += 1;
    }
    let method_end = pos;
    while (buf[pos] == b' ') {
        pos += 1;
    }
    let path_start = pos;
    while (buf[pos] != b' ') {
        pos += 1;
    }
    let path_end = pos;

    Ok((&buf[method_start..method_end], &buf[path_start..path_end]))
}
pub(super) async fn read(socket: &mut TcpSocket<'_>) -> Result<((), usize), Error> {
    let mut buf = [0; 4096];
    let n = match socket.read(&mut buf).await {
        Ok(0) => {
            warn!("read EOF");
            return Err(Error::ReadEof);
        }
        Ok(n) => {
            info!("read {}", n);
            n
        }
        Err(embassy_net::tcp::Error::ConnectionReset) => {
            warn!("read error: ConnectionReset");
            return Err(Error::ConnectionReset);
        }
    };
    let mut headers = [httparse::EMPTY_HEADER; 16];
    let (body_start, content_length, ..) = parse(&buf, n, &mut headers)?;
    // let mut req = httparse::Request::new(&mut headers);
    // let (body_start, content_length) = match req.parse(&buf[..n]) {
    //     Ok(httparse::Status::Complete(p)) => {
    //         info!("Complete response: {} {}", req.method, req.path);
    //         for i in 0..req.headers.len() {
    //             debug!(
    //                 "header {}: {} -> {}",
    //                 i,
    //                 req.headers[i].name,
    //                 core::str::from_utf8(req.headers[i].value).unwrap()
    //             );
    //         }
    //         debug!("Body starts at: {}", p);
    //         info!(
    //             "Body: (len: {}) starts: {}",
    //             &buf[p..n].len(),
    //             core::str::from_utf8(&buf[p..(p + 5)]).unwrap()
    //         );

    //         for content_length in find_header(&headers, "CONTENT-LENGTH") {
    //             debug!("56: content-length: {}", content_length.value);
    //             let value = core::str::from_utf8(content_length.value)?.parse::<u64>()?;
    //             debug!("    value: {}", value);
    //         }

    //         let mut content_lengths = find_header(&headers, "CONTENT-LENGTH");
    //         let content_length = if let Some(content_length_header) = content_lengths.next() {
    //             core::str::from_utf8(content_length_header.value)?.parse::<usize>()?
    //         } else {
    //             return Err(Error::UnknownContentLength);
    //         };
    //         if content_lengths.next().is_none() {
    //             (p, content_length)
    //         } else {
    //             return Err(Error::MultipleContentLengths);
    //         }
    //     }
    //     Ok(httparse::Status::Partial) => {
    //         error!("Partial response: {} {}", req.method, req.path);
    //         for i in 0..req.headers.len() {
    //             error!(
    //                 "header {}: {} -> {}",
    //                 i,
    //                 req.headers[i].name,
    //                 core::str::from_utf8(req.headers[i].value).unwrap()
    //             );
    //         }
    //         return Err(Error::IncompleteHeaderRead);
    //     }
    //     Err(e) => {
    //         error!("Header parsing failed: {}", e.to_string());
    //         return Err(Error::HttpParse(e));
    //     }
    // };

    for (i, l) in buf[..n].split(|&c| c == b'\n').enumerate() {
        let line = core::str::from_utf8(l).unwrap();
        info!("rxd line {} (len {}): {}", i, line.len(), line);
    }

    for (i, l) in buf[body_start..core::cmp::min(body_start + content_length, n)]
        .split(|&c| c == b'\n')
        .enumerate()
    {
        let line = core::str::from_utf8(l).unwrap();
        info!("body line {} (len {}): {}", i, line.len(), line);
    }

    let (method, path) = parse_request_line(&buf)?;
    info!("Complete response: {} {}", method, path);
    info!(
        "    {} {}",
        core::str::from_utf8(method).unwrap(),
        core::str::from_utf8(path).unwrap()
    );
    match method {
        b"PROPFIND" => {
            info!("PROPFIND");
        }
        _ => {
            info!("Unhandled");
        }
    }

    // TODO Process request body
    let body = ();
    Ok((body, content_length))
}
