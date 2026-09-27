// Command attitude-server runs the rigid-body attitude quaternion integration
// HTTP service. It exposes no pages — only a JSON API.
package main

import (
	"log"
	"net/http"
	"os"
	"time"

	"github.com/example/attitude-service/internal/api"
	"github.com/example/attitude-service/internal/store"
)

func main() {
	addr := os.Getenv("ATTITUDE_HTTP_ADDR")
	if addr == "" {
		addr = ":8080"
	}

	srv := &http.Server{
		Addr:              addr,
		Handler:           api.NewServer(store.New()).Router(),
		ReadHeaderTimeout: 10 * time.Second,
	}
	log.Printf("attitude quaternion integration service listening on %s", addr)
	if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		log.Fatalf("server error: %v", err)
	}
}
