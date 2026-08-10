package executioncatalog

import (
	"crypto/sha256"
	"encoding/hex"
	"os"
	"path/filepath"
	"strings"
	"testing"

	processsupervisor "xgc2/core-xgc/internal/process"
	"xgc2/core-xgc/internal/processcatalog"
)

func TestExternalSessionClockGuardProductMatchesAdmissionContract(t *testing.T) {
	manifest := strings.TrimSpace(os.Getenv("XGC_SESSION_CLOCK_GUARD_MANIFEST"))
	if manifest == "" {
		t.Fatal("XGC_SESSION_CLOCK_GUARD_MANIFEST is required")
	}
	artifactRoot := t.TempDir()
	policyDirectory := filepath.Join(processcatalog.SessionClockPolicyRoot(artifactRoot), "session-1")
	if err := os.MkdirAll(policyDirectory, 0o750); err != nil {
		t.Fatal(err)
	}
	policyFile := filepath.Join(policyDirectory, "clock.cfg")
	policyBytes := []byte("schema=xgc.session-clock-guard.config.v2\n")
	if err := os.WriteFile(policyFile, policyBytes, 0o440); err != nil {
		t.Fatal(err)
	}
	sum := sha256.Sum256(policyBytes)

	registry := processsupervisor.NewRegistry()
	if err := processcatalog.InstallRuntime(
		registry, processcatalog.RuntimeConfig{ArtifactRoot: artifactRoot},
	); err != nil {
		t.Fatal(err)
	}
	if err := processsupervisor.LoadDefinitionPlugins(registry, []string{manifest}); err != nil {
		t.Fatal(err)
	}
	definition, found := registry.Get(processcatalog.SessionClockGuardDefinitionID)
	if !found {
		t.Fatal("formal Session Clock Guard definition was not discovered")
	}
	parameters, err := registry.NormalizeParameters(definition, map[string]any{
		"policyFile": policyFile, "policySha256": hex.EncodeToString(sum[:]), "epochId": "1",
	})
	if err != nil {
		t.Fatal(err)
	}
	if err := validateSessionClockGuardDefinitionContract(definition, parameters); err != nil {
		t.Fatalf("formal product definition is not admissible: %v", err)
	}
}
