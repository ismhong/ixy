ARG ALPINE_VERSION=3.21
ARG RUNTIME_ARCH=arm64v8

FROM ${RUNTIME_ARCH}/alpine:${ALPINE_VERSION}
RUN apk update && apk add --no-cache alpine-sdk \
                                     linux-headers \
                                     cmake \
                                     ninja

WORKDIR /working

COPY build.sh /build.sh
RUN chmod 755 /build.sh
ENTRYPOINT ["/bin/ash", "/build.sh"] 
