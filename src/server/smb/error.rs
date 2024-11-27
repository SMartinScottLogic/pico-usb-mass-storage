use defmt::{write, Format, Formatter};
pub(super) enum Error {
    ReadEof,
    ConnectionReset,
    // IncompleteHeaderRead,
    // MissingHttpMethod,
    // UnknownHttpMethod,
    // UnknownContentLength,
    // InvalidContentLength,
    // MultipleContentLengths,
    // InvalidPath,
    // HttpParse(httparse::Error),
    // Utf8(usize, Option<usize>),
    // ParseInt(IntErrorKind),
    // CoreIo,
}
impl Format for Error {
    fn format(&self, fmt: Formatter) {
        use Error::*;
        match self {
            ReadEof => write!(fmt, "ReadEof"),
            ConnectionReset => write!(fmt, "ConnectionReset"),
            // IncompleteHeaderRead => write!(fmt, "IncompleteHeaderRead"),
            // MissingHttpMethod => write!(fmt, "MissingHttpMethod"),
            // UnknownHttpMethod => write!(fmt, "UnknownHttpMethod"),
            // UnknownContentLength => write!(fmt, "UnknownContentLength"),
            // InvalidContentLength => write!(fmt, "InvalidContentLength"),
            // MultipleContentLengths => write!(fmt, "MultipleContentLengths"),
            // InvalidPath => write!(fmt, "InvalidPath"),
            // HttpParse(error) => write!(fmt, "ParseError({})", error.to_string()),
            // Utf8(a, b) => write!(fmt, "Utf8Error({}, {:?})", a, b),
            // ParseInt(int_error_kind) => {
            //     write!(fmt, "ParseIntError({})", int_error_kind.to_string())
            // }
            // CoreIo => write!(fmt, "Core2IoError"),
        }
    }
}
