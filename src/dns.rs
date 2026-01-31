use std::net::{Ipv4Addr, UdpSocket};

const DNS_PORT: u16 = 53;
const DNS_MAX_LEN: usize = 256;

const OPCODE_MASK: u16 = 0x7800;
const QR_FLAG: u16 = 1 << 7;
const QD_TYPE_A: u16 = 1;
const ANS_TTL_SEC: usize = 300;

const TAG: &'static str = "captive_dns_redirect_server";

/// DNS Header Packet
#[repr(packed)]
struct DnsHeader {
    id: u16,
    flags: u16,
    qd_count: u16,
    an_count: u16,
    ns_count: u16,
    ar_count: u16,
}

/// DNS Question Packet
#[repr(packed)]
struct DnsQuestion {
    type_: u16,
    class: u16,
}

/// DNS Answer Packet
#[repr(packed)]
struct DnsAnswer {
    ptr_offset: u16,
    type_: u16,
    class: u16,
    ttl: u16,
    addr_len: u16,
    ip_addr: u16,
}

pub(crate) fn dns_server(ip: Ipv4Addr) -> anyhow::Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:53")?;
    let mut buf = [0u8; 512];
    loop {
        let (len, src) = socket.recv_from(&mut buf)?;
        if let Some(resp) = build_dns_response(&buf[..len], ip) {
            socket.send_to(&resp, src)?;
        }
    }
}

fn build_dns_response(req: &[u8], ip: Ipv4Addr) -> Option<Vec<u8>> {
    log::info!("handle dns response");
    if req.len() < 12 {
        return None;
    }

    let mut response = Vec::with_capacity(512);

    // transaction id
    response.extend_from_slice(&req[0..2]);

    // flags
    response.extend_from_slice(&[0x81, 0x80]);

    // QDCOUNT = 1
    response.extend_from_slice(&[0x00, 0x01]);

    // ANCOUNT = 1
    response.extend_from_slice(&[0x00, 0x01]);

    // NSCOUNT, ARCOUNT = 0
    response.extend_from_slice(&[0x00, 0x00, 0x00, 0x00]);

    // Question
    let mut idx = 12;
    while idx < req.len() && req[idx] != 0 {
        idx += 1;
    }
    idx += 5; // null label + QTYPE + QCLASS
    if idx > req.len() {
        return None;
    }

    log::info!("req: {}", str::from_utf8(&req[12..idx]).unwrap());
    response.extend_from_slice(&req[12..idx]);

    // Answer sectoin
    response.extend_from_slice(&[0xC0, 0x0C]);

    // TYPE A, CLASS IN
    response.extend_from_slice(&[0x00, 0x01, 0x00, 0x01]);

    // TTL = 60 seconds
    response.extend_from_slice(&[0x00, 0x00, 0x00, 0x3c]);

    // RDLENGTH = 4
    response.extend_from_slice(&[0x00, 0x04]);

    // RDATA = AP IP
    response.extend_from_slice(ip.as_octets().as_slice());
    Some(response)
}
