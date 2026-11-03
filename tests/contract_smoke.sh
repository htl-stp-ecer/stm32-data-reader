#!/usr/bin/env bash
# The real reader binary must refuse to start (exit 78) when its transport contract does
# not match the bundle file, and must not refuse in dev mode or off a Raccoon device.
set -u
reader="$1"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
mkdir "$tmp/image"
export RACCOON_STATUS_DIR="$tmp/run" RACCOON_BUNDLE_FILE="$tmp/bundle.json"
fail=0
version=$("$reader" --version)
hash=1111111111111111

expect() { # name, expected exit code (124 = still running when the timeout hit), device-marker dir
    local name="$1" want="$2" marker="$3"
    RACCOON_DEVICE_MARKER="$marker" timeout 2 "$reader" >"$tmp/out" 2>&1
    local got=$?
    if [ "$got" != "$want" ]; then
        echo "FAIL $name: exit $got, expected $want"; sed 's/^/    /' "$tmp/out" | head -5; fail=1
    else
        echo "ok   $name"
    fi
}

bundle() { # contract_hash, this reader's entry in the bundle
    printf '{"bundle":"b","contract_hash":"%s","components":{"stm32-data-reader":"%s"}}' "$1" "$2" > "$tmp/bundle.json"
}
# The hash the binary was built with, read the way the image build reads it.
own=$(strings -a "$reader" | grep -o 'RACCOON_CONTRACT_V1=[0-9a-f]*' | head -1 | cut -d= -f2)

bundle "$hash" "$version"
expect "mismatch refuses" 78 "$tmp/image"
if ! grep -q '"result": "mismatch"' "$tmp/run/stm32-data-reader.json" 2>/dev/null; then
    echo "FAIL mismatch writes a status file"; fail=1
fi
rm "$tmp/bundle.json"
expect "missing bundle file refuses" 78 "$tmp/image"
echo '{"contract_hash":""}' > "$tmp/bundle.json"
expect "empty contract_hash refuses" 78 "$tmp/image"
echo '{"contract_hash":"dev"}' > "$tmp/bundle.json"
expect "dev contract without a version entry refuses" 78 "$tmp/image"
bundle dev "$version"
expect "dev mode keeps running" 124 "$tmp/image"
bundle "$own" "$version"
expect "matching hash and version keeps running" 124 "$tmp/image"
bundle "$own" "v$version"
expect "version with v prefix keeps running" 124 "$tmp/image"
bundle "$own" "99999"
expect "wrong version refuses" 78 "$tmp/image"
bundle "$own" "dev"
expect "dev version entry keeps running" 124 "$tmp/image"
rm "$tmp/bundle.json"
expect "off a device keeps running" 124 "$tmp/no-such-dir"
exit $fail
