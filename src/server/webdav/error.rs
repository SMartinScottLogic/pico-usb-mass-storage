use core::{
    num::{IntErrorKind, ParseIntError},
    str::Utf8Error,
};
use defmt::{write, Format, Formatter};

pub(super) enum Error {
    ReadEof,
    ConnectionReset,
    IncompleteHeaderRead,
    MissingHttpMethod,
    UnknownHttpMethod,
    UnknownContentLength,
    InvalidContentLength,
    MultipleContentLengths,
    InvalidPath,
    HttpParse(httparse::Error),
    Utf8(usize, Option<usize>),
    ParseInt(IntErrorKind),
    CoreIo,
}
impl From<Utf8Error> for Error {
    fn from(value: Utf8Error) -> Self {
        Self::Utf8(value.valid_up_to(), value.error_len())
    }
}
impl From<ParseIntError> for Error {
    fn from(value: ParseIntError) -> Self {
        Self::ParseInt(value.kind().clone())
    }
}
impl From<core2::io::Error> for Error {
    fn from(_value: core2::io::Error) -> Self {
        Self::CoreIo
    }
}
impl Format for Error {
    fn format(&self, fmt: Formatter) {
        use Error::*;
        match self {
            ReadEof => write!(fmt, "ReadEof"),
            ConnectionReset => write!(fmt, "ConnectionReset"),
            IncompleteHeaderRead => write!(fmt, "IncompleteHeaderRead"),
            MissingHttpMethod => write!(fmt, "MissingHttpMethod"),
            UnknownHttpMethod => write!(fmt, "UnknownHttpMethod"),
            UnknownContentLength => write!(fmt, "UnknownContentLength"),
            InvalidContentLength => write!(fmt, "InvalidContentLength"),
            MultipleContentLengths => write!(fmt, "MultipleContentLengths"),
            InvalidPath => write!(fmt, "InvalidPath"),
            HttpParse(error) => write!(fmt, "ParseError({})", error.to_string()),
            Utf8(a, b) => write!(fmt, "Utf8Error({}, {:?})", a, b),
            ParseInt(int_error_kind) => {
                write!(fmt, "ParseIntError({})", int_error_kind.to_string())
            }
            CoreIo => write!(fmt, "Core2IoError"),
        }
    }
}

pub trait ToStaticString {
    fn to_string(&self) -> &'static str;
}

impl ToStaticString for httparse::Error {
    fn to_string(&self) -> &'static str {
        match &self {
            httparse::Error::HeaderName => "invalid header name",
            httparse::Error::HeaderValue => "invalid header value",
            httparse::Error::NewLine => "invalid new line",
            httparse::Error::Status => "invalid response status",
            httparse::Error::Token => "invalid token",
            httparse::Error::TooManyHeaders => "too many headers",
            httparse::Error::Version => "invalid HTTP version",
        }
    }
}

impl ToStaticString for IntErrorKind {
    fn to_string(&self) -> &'static str {
        match &self {
            IntErrorKind::Empty => "empty",
            IntErrorKind::InvalidDigit => "invalid digit",
            IntErrorKind::PosOverflow => "positive overflow",
            IntErrorKind::NegOverflow => "negative overflow",
            IntErrorKind::Zero => "zero",
            _ => "unknown error",
        }
    }
}
