SSLPATH=/scrap/ilyal/TLS/openssl/

env LD_LIBRARY_PATH=#LD_LIBRARY_PATH:$SSLPATH ./configure LDFLAGS="-L$SSLPATH -lssl -lcrypto"
