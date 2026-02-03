use std::net::{Ipv4Addr, UdpSocket};

const DNS_MAX_LEN: usize = 256;

const OPCODE_MASK: u16 = 0x7800;
const QR_FLAG: u16 = 1 << 15;
const QD_TYPE_A: u16 = 1;
const ANS_TTL_SEC: usize = 60;

/// DNS Header Packet
#[derive(Debug, Clone, Copy)]
struct DnsHeader {
    id: u16,
    flags: u16,
    qd_count: u16,
    an_count: u16,
    ns_count: u16,
    ar_count: u16,
}

fn read_u16(buf: &mut &[u8]) -> u16 {
    let mut bytes = [0u8; 2];
    bytes.copy_from_slice(&buf[..2]);
    *buf = &buf[2..];
    u16::from_be_bytes(bytes)
}

impl DnsHeader {
    fn from_bytes(buf: &mut &[u8]) -> Option<Self> {
        if buf.len() < size_of::<Self>() {
            None
        } else {
            Some(Self {
                id: read_u16(buf),
                flags: read_u16(buf),
                qd_count: read_u16(buf),
                an_count: read_u16(buf),
                ns_count: read_u16(buf),
                ar_count: read_u16(buf),
            })
        }
    }

    fn to_be_bytes(&self) -> [u8; size_of::<Self>()] {
        let mut buf = [0u8; size_of::<Self>()];
        let mut res = &mut buf[..];
        res[..2].copy_from_slice(&self.id.to_be_bytes());
        res = &mut res[2..];
        res[..2].copy_from_slice(&self.flags.to_be_bytes());
        res = &mut res[2..];
        res[..2].copy_from_slice(&self.qd_count.to_be_bytes());
        res = &mut res[2..];
        res[..2].copy_from_slice(&self.an_count.to_be_bytes());
        res = &mut res[2..];
        res[..2].copy_from_slice(&self.ns_count.to_be_bytes());
        res = &mut res[2..];
        res[..2].copy_from_slice(&self.ar_count.to_be_bytes());
        buf
    }
}

/// DNS Question Packet
#[derive(Debug, Clone, Copy)]
struct DnsQuestion {
    type_: u16,
    class: u16,
}

impl DnsQuestion {
    fn from_bytes(buf: &mut &[u8]) -> Option<Self> {
        if buf.len() < size_of::<Self>() {
            None
        } else {
            Some(Self {
                type_: read_u16(buf),
                class: read_u16(buf),
            })
        }
    }
}

/// DNS Answer Packet
#[derive(Debug)]
struct DnsAnswer {
    ptr_offset: u16,
    type_: u16,
    class: u16,
    ttl: u32,
    addr_len: u16,
    ip_addr: u32,
}

impl DnsAnswer {
    fn to_be_bytes(&self) -> [u8; size_of::<Self>()] {
        let mut buf = [0u8; size_of::<Self>()];
        let mut res = &mut buf[..];
        res[..2].copy_from_slice(&self.ptr_offset.to_be_bytes());
        res = &mut res[2..];
        res[..2].copy_from_slice(&self.type_.to_be_bytes());
        res = &mut res[2..];
        res[..2].copy_from_slice(&self.class.to_be_bytes());
        res = &mut res[2..];
        res[..4].copy_from_slice(&self.ttl.to_be_bytes());
        res = &mut res[4..];
        res[..2].copy_from_slice(&self.addr_len.to_be_bytes());
        res = &mut res[2..];
        res[..4].copy_from_slice(&self.ip_addr.to_be_bytes());
        buf
    }
}

pub(crate) fn dns_server(ip: Ipv4Addr) -> anyhow::Result<()> {
    let socket = UdpSocket::bind("0.0.0.0:53")?;
    let mut req_buf = [0u8; 128];
    loop {
        let (len, src) = match socket.recv_from(&mut req_buf) {
            Ok((0, _)) => {
                log::warn!("dns: empty requeset");
                continue;
            }
            Ok(d) => d,
            Err(e) => {
                log::warn!("dns recv_from(): {e}");
                continue;
            }
        };
        let req = &req_buf[..len];
        let mut resp_buf = vec![0u8; DNS_MAX_LEN];
        if let Some(resp) = build_dns_response(req, ip, &mut resp_buf) {
            if let Err(e) = socket.send_to(&resp, src) {
                log::warn!("dns send_to(): {e}");
            }
        }
    }
}

fn write_bytes(dst: &mut [u8], offset: usize, src: &[u8]) -> usize {
    dst[offset..offset + src.len()].copy_from_slice(src);
    src.len()
}

fn parse_dns_name<'a>(req: &mut &[u8], dst: &'a mut [u8]) -> Option<(usize, &'a str)> {
    let mut len = 0;
    loop {
        let sub_name_len = req[0] as usize;
        *req = &req[1..];
        len += 1;
        if sub_name_len == 0 {
            break;
        }

        assert!(
            sub_name_len <= req.len(),
            "sn len: {sub_name_len}, req.len(): {}",
            req.len()
        );

        if dst.len() < len + sub_name_len {
            return None;
        }

        dst[len..len + sub_name_len].copy_from_slice(&req[..sub_name_len]);
        dst[len + sub_name_len] = b'.';
        *req = &req[sub_name_len..];
        len += sub_name_len;
    }
    Some((len, str::from_utf8(&dst[..len]).ok()?))
}

fn build_dns_response<'a>(mut req: &[u8], ip: Ipv4Addr, buf: &'a mut [u8]) -> Option<&'a [u8]> {
    let req_len = req.len();
    if req_len > buf.len() {
        log::warn!("dns: request too long, skipping");
        return None;
    }

    let header = DnsHeader::from_bytes(&mut req)?;
    log::info!("dns query: {header:?}");

    if header.flags & OPCODE_MASK != 0 {
        log::warn!("dns: non standard query");
        return None;
    }

    let mut response_header = header;
    response_header.flags |= QR_FLAG;
    response_header.an_count = header.qd_count;
    response_header.ar_count = 0;
    response_header.ns_count = 0;
    buf[..size_of::<DnsHeader>()].copy_from_slice(&response_header.to_be_bytes());

    let resp_len = {
        let mut total = 0;
        let mut req = req;
        let mut buf = [0u8; 128];
        for _ in 0..header.qd_count {
            let (size, _) = parse_dns_name(&mut req, &mut buf)?; // name
            let _ = DnsQuestion::from_bytes(&mut req); // question
            total += size + size_of::<DnsQuestion>();
        }
        total
    };
    log::info!("req len: {resp_len}");
    let reply_len = size_of::<DnsHeader>()
        + resp_len
        + response_header.an_count as usize * size_of::<DnsAnswer>();
    if reply_len > buf.len() {
        return None;
    }

    buf[size_of::<DnsHeader>()..size_of::<DnsHeader>() + resp_len]
        .copy_from_slice(&req[..resp_len]);

    let mut src_idx = size_of::<DnsHeader>();
    let mut dst_idx = size_of::<DnsHeader>() + resp_len;
    for _ in 0..header.qd_count {
        let mut name_buf = [0u8; 128];
        let (size, name) = parse_dns_name(&mut req, &mut name_buf)?;
        let question = DnsQuestion::from_bytes(&mut req)?;
        log::info!("Q: {name} {question:?}");
        let qd_type = question.type_;
        if qd_type == QD_TYPE_A {
            let answer = DnsAnswer {
                ptr_offset: (0xC000 | src_idx as u16),
                type_: question.type_,
                class: question.class,
                ttl: ANS_TTL_SEC as u32,
                addr_len: size_of::<Ipv4Addr>() as u16,
                ip_addr: ip.to_bits(),
            };
            log::info!("A: {answer:x?}");
            dst_idx += write_bytes(buf, dst_idx, &answer.to_be_bytes());
        }
        src_idx += size;
    }

    Some(&buf[..reply_len])
}
