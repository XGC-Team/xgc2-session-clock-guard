package processcatalog

import (
	"crypto/sha256"
	"encoding/hex"
	"os"
	"path/filepath"
	"strings"
	"testing"

	processsupervisor "xgc2/core-xgc/internal/process"
)

func TestExternalSessionClockGuardProductDiscovery(t *testing.T) {
	manifest := strings.TrimSpace(os.Getenv("XGC_SESSION_CLOCK_GUARD_MANIFEST"))
	if manifest == "" {
		t.Fatal("XGC_SESSION_CLOCK_GUARD_MANIFEST is required")
	}
	artifactRoot := t.TempDir()
	policyDirectory := filepath.Join(SessionClockPolicyRoot(artifactRoot), "session-1")
	if err := os.MkdirAll(policyDirectory, 0o750); err != nil {
		t.Fatal(err)
	}
	policyFile := filepath.Join(policyDirectory, "clock.cfg")
	policyBytes := []byte("schema=xgc.session-clock-guard.config.v2\n")
	if err := os.WriteFile(policyFile, policyBytes, 0o440); err != nil {
		t.Fatal(err)
	}
	sum := sha256.Sum256(policyBytes)
	policyDigest := hex.EncodeToString(sum[:])

	registry := processsupervisor.NewRegistry()
	if err := InstallRuntime(registry, RuntimeConfig{ArtifactRoot: artifactRoot}); err != nil {
		t.Fatalf("Core Process policies rejected product setup: %v", err)
	}
	if err := processsupervisor.LoadDefinitionPlugins(registry, []string{manifest}); err != nil {
		t.Fatalf("Core Process loader rejected product manifest: %v", err)
	}
	definition, found := registry.Get("xgc2-session-clock-guard")
	if !found || definition.Digest == "" || definition.Version != "0.1.0" {
		t.Fatalf("discovered definition is not normalized and digest-pinned: %+v", definition)
	}
	if len(definition.Drivers) != 1 || definition.Drivers[0] != "host" {
		t.Fatalf("definition is not Core-local host-only: %#v", definition.Drivers)
	}
	if !definition.Internal {
		t.Fatal("Session Clock Guard definition is public; generic process.run-definition could start it")
	}
	parameters, err := registry.NormalizeParameters(definition, map[string]any{
		"policyFile":   policyFile,
		"policySha256": policyDigest,
		"epochId":      "18446744073709551615",
	})
	if err != nil {
		t.Fatalf("required frozen parameters did not normalize: %v", err)
	}
	if _, found := parameters["threshold.maxJitterNs"]; found {
		t.Fatal("workflow-visible threshold bypassed the frozen policy file")
	}
	claims, err := definition.ResolveResourceClaims(parameters)
	if err != nil {
		t.Fatalf("resource claims did not resolve: %v", err)
	}
	if len(claims) != 5 {
		t.Fatalf("resolved resource claim count=%d, want 5", len(claims))
	}
	modes := map[string]processsupervisor.ResourceAccessMode{}
	for _, claim := range claims {
		modes[claim.BindingKey] = claim.Mode
	}
	if modes["canonical-vrpn-root"] != processsupervisor.ResourceAccessExclusive ||
		modes["session-clock-sidecar-root"] != processsupervisor.ResourceAccessExclusive ||
		modes["simulation-raw-vrpn-root"] != processsupervisor.ResourceAccessShared ||
		modes["physical-raw-vrpn-root"] != processsupervisor.ResourceAccessShared {
		t.Fatalf("resolved ownership modes are wrong: %#v", modes)
	}
	if definition.Readiness.Kind != processsupervisor.ProbeExec ||
		!strings.HasSuffix(definition.Readiness.Command.Executable, "/session_clock_guard_healthcheck") ||
		len(definition.Readiness.Command.Args) != 3 ||
		definition.Readiness.Timeout != 5_000_000_000 {
		t.Fatalf("readiness is not the digest/epoch-bound locked-state checker: %+v", definition.Readiness)
	}
	if definition.Liveness.Kind != processsupervisor.ProbeExec ||
		definition.Liveness.Command.Executable != definition.Readiness.Command.Executable ||
		len(definition.Liveness.Command.Args) != 3 ||
		definition.Liveness.Interval != 1_000_000_000 ||
		definition.Liveness.Timeout != 5_000_000_000 ||
		definition.Liveness.SuccessThreshold != 1 ||
		definition.Liveness.FailureThreshold != 1 {
		t.Fatalf("liveness is not the prompt strong locked-state checker: %+v", definition.Liveness)
	}
}
