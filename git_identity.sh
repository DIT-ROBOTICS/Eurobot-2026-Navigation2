#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  ./git_identity.sh <github_username> [email]

Examples:
  ./git_identity.sh 2006wu
  ./git_identity.sh 2006wu wuchengzhi20060819@gmail.com

If email is omitted, the script uses GitHub's noreply format:
  <username>@users.noreply.github.com
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  usage
  exit 1
fi

USERNAME="$1"
EMAIL="${2:-${USERNAME}@users.noreply.github.com}"

git config user.name "$USERNAME"
git config user.email "$EMAIL"

echo "Configured git identity:"
echo "  user.name  = $USERNAME"
echo "  user.email = $EMAIL"
