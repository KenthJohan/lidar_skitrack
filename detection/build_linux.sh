gcc pointfile.c -odetection_linux64 \
-DNNG_STATIC_LIB \
-I../csc \
-lnng \
-lopenblas \
-lm