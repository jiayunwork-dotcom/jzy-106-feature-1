# syntax=docker/dockerfile:1

# ---- build stage -------------------------------------------------------------
FROM golang:1.22-alpine AS build

# Override with --build-arg GOPROXY=https://goproxy.cn,direct if your network
# cannot reach proxy.golang.org.
ARG GOPROXY=https://proxy.golang.org,direct
ENV GOPROXY=${GOPROXY} \
    CGO_ENABLED=0 \
    GOOS=linux

WORKDIR /src

# Cache module downloads separately from source changes.
COPY go.mod go.sum ./
RUN go mod download

COPY . .
RUN go build -trimpath -ldflags="-s -w" -o /out/attitude-server ./cmd/server

# ---- runtime stage -----------------------------------------------------------
FROM alpine:3.20

RUN addgroup -S attitude && adduser -S attitude -G attitude \
    && apk add --no-cache wget

COPY --from=build /out/attitude-server /usr/local/bin/attitude-server

USER attitude
EXPOSE 8080

HEALTHCHECK --interval=10s --timeout=2s --start-period=3s --retries=3 \
    CMD wget -qO- http://127.0.0.1:8080/health >/dev/null 2>&1 || exit 1

ENTRYPOINT ["/usr/local/bin/attitude-server"]
