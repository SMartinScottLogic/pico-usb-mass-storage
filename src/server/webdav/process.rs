use crate::fat12_partition;

use super::error::Error;
use super::error::ToStaticString as _;
use core2::io::Write as _;
use defmt::{error, info, warn};
use embassy_net::tcp::TcpSocket;
use embedded_io_async::Write as _;
use httparse::Header;

fn cstr_len(buf: &[u8]) -> usize {
    let mut i = 0;
    loop {
        if buf[i] == 0 {
            break i;
        }
        i += 1;
    }
}

fn find_header<'a, 'b>(
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

pub async fn process<const BUFFER_SIZE: usize, const NUM_OF_HEADERS: usize>(
    socket: &mut TcpSocket<'_>,
) -> Result<(), Error> {
    let processor = Processor::<BUFFER_SIZE, NUM_OF_HEADERS>;
    // 1. Read
    let mut buf = [0; BUFFER_SIZE];
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
    // 2. Parse buffer
    let mut headers = [httparse::EMPTY_HEADER; NUM_OF_HEADERS];
    let mut req = httparse::Request::new(&mut headers);
    match req.parse(&buf[..n]) {
        Err(e) => {
            error!("Header parsing failed: {}", e.to_string());
            Err(Error::HttpParse(e))
        }
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
        Ok(httparse::Status::Complete(p)) => match req.method {
            Some(method) => {
                processor
                    .handle_request(socket, method, req.path, p, &buf, &headers)
                    .await
            }
            None => {
                error!("Invalid HTTP method");
                Err(Error::MissingHttpMethod)
            }
        },
    }
}

pub struct Processor<const BUFFER_SIZE: usize, const NUM_OF_HEADERS: usize>;
impl<const BUFFER_SIZE: usize, const NUM_OF_HEADERS: usize> Processor<BUFFER_SIZE, NUM_OF_HEADERS> {
    async fn handle_request(
        &self,
        socket: &mut TcpSocket<'_>,
        method: &str,
        path: Option<&str>,
        _request_body_start: usize,
        _request_buf: &[u8],
        headers: &[httparse::Header<'_>],
    ) -> Result<(), Error> {
        info!("handle_request: {} {}", method, path);
        let content_length = find_header(headers, "CONTENT-LENGTH").next();
        match path {
            None => {
                error!("{} with no path", method);
                Err(Error::InvalidPath)
            }
            Some(path) => match method {
                "PROPFIND" => self.propfind(socket, path).await,
                "GET" if content_length.is_some() => {
                    error!("GET doesn't support request content");
                    Err(Error::InvalidContentLength)
                }
                "GET" => self.get(socket, path).await,
                "PUT" => self.put(socket, path).await,
                "DELETE" => self.delete(socket, path).await,
                _ => {
                    error!("Unknown method {}", method);
                    Err(Error::UnknownHttpMethod)
                }
            },
        }
    }

    async fn get(&self, socket: &mut TcpSocket<'_>, path: &str) -> Result<(), Error> {
        let mut response_body = [0; BUFFER_SIZE];
        let response_body_size = {
            #[allow(static_mut_refs)]
            let storage = unsafe { &mut crate::STORAGE };

            #[allow(static_mut_refs)]
            fat12_partition::log_fs(
                storage.as_bytes_mut(),
                crate::storage::BLOCKS as _,
                crate::storage::BLOCK_SIZE as _,
            );
            fat12_partition::read(
                storage.as_bytes_mut(),
                crate::storage::BLOCKS as _,
                crate::storage::BLOCK_SIZE as _,
                path,
                &mut response_body,
            )
            .map_err(|_| Error::ReadEof)?
        };
        let mut headers = [0; BUFFER_SIZE];
        let mut cur = core2::io::Cursor::new(headers.as_mut_slice());
        write!(
            cur,
            "HTTP/1.1 200 Ok\r\nContent-Type: text/plain\r\nContent-Length: {}",
            response_body_size
        )?;
        // Headers
        socket
            .write_all(&headers[..cstr_len(&headers)])
            .await
            .map_err(|_err| Error::ConnectionReset)?;
        // Separator
        socket
            .write_all(b"\r\n\r\n")
            .await
            .map_err(|_err| Error::ConnectionReset)?;
        // Body
        socket
            .write_all(&response_body)
            .await
            .map_err(|_err| Error::ConnectionReset)?;
        Ok(())
    }

    async fn put(&self, socket: &mut TcpSocket<'_>, _path: &str) -> Result<(), Error> {
        let mut headers = [0; BUFFER_SIZE];
        let mut cur = core2::io::Cursor::new(headers.as_mut_slice());
        write!(
            cur,
            "HTTP/1.1 200 Ok\r\nContent-Type: text/plain\r\nContent-Length: 0"
        )?;
        // Headers
        socket
            .write_all(&headers[..cstr_len(&headers)])
            .await
            .map_err(|_err| Error::ConnectionReset)?;
        // Separator
        socket
            .write_all(b"\r\n\r\n")
            .await
            .map_err(|_err| Error::ConnectionReset)?;
        Ok(())
    }

    async fn delete(&self, socket: &mut TcpSocket<'_>, _path: &str) -> Result<(), Error> {
        let mut headers = [0; BUFFER_SIZE];
        let mut cur = core2::io::Cursor::new(headers.as_mut_slice());
        write!(
            cur,
            "HTTP/1.1 200 Ok\r\nContent-Type: text/plain\r\nContent-Length: 0"
        )?;
        // Headers
        socket
            .write_all(&headers[..cstr_len(&headers)])
            .await
            .map_err(|_err| Error::ConnectionReset)?;
        // Separator
        socket
            .write_all(b"\r\n\r\n")
            .await
            .map_err(|_err| Error::ConnectionReset)?;
        Ok(())
    }

    async fn propfind(&self, socket: &mut TcpSocket<'_>, _path: &str) -> Result<(), Error> {
        let body = b"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<multistatus xmlns=\"DAV:\">\r\n <response>\r\n  <href>/</href>\r\n </response>\r\n</multistatus>\r\n\r\n";
        let mut headers = [0; BUFFER_SIZE];
        let mut cur = core2::io::Cursor::new(headers.as_mut_slice());
        write!(
            cur,
            "HTTP/1.1 207 Multi-Status\r\nContent-Type: application/xml; charset=\"utf-8\"\r\nContent-Length: {}", 
            body.len()
        )?;

        // Headers
        socket
            .write_all(&headers[..cstr_len(&headers)])
            .await
            .map_err(|_err| Error::ConnectionReset)?;
        // Separator
        socket
            .write_all(b"\r\n\r\n")
            .await
            .map_err(|_err| Error::ConnectionReset)?;
        // Body
        socket
            .write_all(body)
            .await
            .map_err(|_err| Error::ConnectionReset)
    }
}
