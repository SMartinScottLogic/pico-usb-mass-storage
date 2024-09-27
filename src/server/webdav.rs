use defmt::{error, info, warn};
use embassy_net::tcp::TcpSocket;
use embedded_io_async::Write as _;
use httparse::Header;

use super::SocketServer;

pub struct Server {}

impl Server {
    pub fn new() -> Self {
        Self {}
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
        .filter(move |header| header.name == header_name)
}

impl SocketServer for Server {
    async fn run(&mut self, mut socket: TcpSocket<'_>) {
        let mut buf = [0; 4096];
        loop {
            let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    warn!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    warn!("read error: {:?}", e);
                    break;
                }
            };
            info!("rxd {}: {}", n, core::str::from_utf8(&buf[..n]).unwrap());
            let mut headers = [httparse::EMPTY_HEADER; 16];
            let mut req = httparse::Request::new(&mut headers);
            match req.parse(&buf[..n]) {
                Ok(httparse::Status::Complete(p)) => {
                    info!("Complete response: {} {}", req.method, req.path);
                    for i in 0..req.headers.len() {
                        info!(
                            "header {}: {} -> {}",
                            i,
                            req.headers[i].name,
                            core::str::from_utf8(req.headers[i].value).unwrap()
                        );
                    }
                    info!("Body starts at: {}", p);
                    info!(
                        "Body: (len: {}) starts: {}",
                        &buf[p..n].len(),
                        core::str::from_utf8(&buf[p..(p + 5)]).unwrap()
                    );
                    for content_length in find_header(&headers, "CONTENT_LENGTH") {
                        info!("content-length: {}", content_length.value);
                    }
                }
                Ok(httparse::Status::Partial) => {
                    info!("Partial response: {} {}", req.method, req.path);
                    for i in 0..req.headers.len() {
                        info!(
                            "header {}: {} -> {}",
                            i,
                            req.headers[i].name,
                            core::str::from_utf8(req.headers[i].value).unwrap()
                        );
                    }
                }
                Err(e) => {
                    let e = match e {
                        httparse::Error::HeaderName => "invalid header name",
                        httparse::Error::HeaderValue => "invalid header value",
                        httparse::Error::NewLine => "invalid new line",
                        httparse::Error::Status => "invalid response status",
                        httparse::Error::Token => "invalid token",
                        httparse::Error::TooManyHeaders => "too many headers",
                        httparse::Error::Version => "invalid HTTP version",
                    };
                    error!("Header parsing failed: {}", e);
                }
            };

            for l in buf[..n].split(|&c| c == b'\n') {
                let line = core::str::from_utf8(l).unwrap();
                info!("rxd line (len {}): {}", line.len(), line);
            }

            // TODO Process (partial) request body + respond iff complete
        }

        let body = b"<?xml version=\"1.0\" encoding=\"UTF-8\"?>\r\n<multistatus xmlns=\"DAV:\">\r\n <response>\r\n  <href>/</href>\r\n </response>\r\n</multistatus>\r\n\r\n";
        let headers =
            b"HTTP/1.1 207 Multi-Status\r\nContent-Type: application/xml; charset=\"utf-8\"";
        // Headers
        match socket.write_all(headers).await {
            Ok(()) => {}
            Err(e) => {
                error!("write error: {:?}", e);
            }
        };
        // Separator
        match socket.write_all(b"\r\n\r\n").await {
            Ok(()) => {}
            Err(e) => {
                error!("write error: {:?}", e);
            }
        }
        // Body
        match socket.write_all(body).await {
            Ok(()) => {}
            Err(e) => {
                error!("write error: {:?}", e);
            }
        };
        socket.close();
    }
}
