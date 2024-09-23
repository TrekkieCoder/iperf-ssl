if [[ $# -eq 2 ]]; then
  ## Argument should have SSL path should be where ssl3.0 is installed
  SSL_LPATH=$1/lib64
  SSL_IPATH=$1/include/
  LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SSL_LPATH ./configure CFLAGS="-I$SSL_IPATH" CPPFLAGS="-I$SSL_IPATH" LDFLAGS="-L$SSL_LPATH -Wl,-rpath=$SSL_LPATH"
else
  ./configure
fi
sed -i '\|#define bool int|d' config.h
make
