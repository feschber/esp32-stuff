use esp_idf_svc::{
    http::{server::EspHttpServer, Method},
    io::Write,
};

const STACK_SIZE: usize = 10240;
const INDEX_HTML: &str = include_str!("captive.html");

pub(crate) fn host_server<'a>() -> anyhow::Result<EspHttpServer<'a>> {
    let mut server = create_server()?;
    server.fn_handler("/*", Method::Get, |req| {
        req.into_ok_response()?
            .write_all(INDEX_HTML.as_bytes())
            .map(|_| ())
    })?;
    Ok(server)
}

fn create_server() -> anyhow::Result<EspHttpServer<'static>> {
    let config = esp_idf_svc::http::server::Configuration {
        stack_size: STACK_SIZE,
        uri_match_wildcard: true,
        ..Default::default()
    };
    Ok(EspHttpServer::new(&config)?)
}
