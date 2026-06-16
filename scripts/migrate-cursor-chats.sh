#!/usr/bin/env bash
# Nach Umbenennung m5-4_LS-Kitchenlights -> LS_CC2500-Lichtsteuerung:
# Cursor-Chats bleiben über Symlinks am alten Pfad erreichbar.
#
# Nutzung (Cursor vorher vollständig beenden):
#   ./scripts/migrate-cursor-chats.sh

set -euo pipefail

WORKSPACES="${HOME}/Workspaces"
OLD_NAME="m5-4_LS-Kitchenlights"
NEW_NAME="LS_CC2500-Lichtsteuerung"
OLD_PATH="${WORKSPACES}/${OLD_NAME}"
NEW_PATH="${WORKSPACES}/${NEW_NAME}"
OLD_WS="${NEW_PATH}/${NEW_NAME}.code-workspace"
CURSOR_PROJ="${HOME}/.cursor/projects"
OLD_PROJ="home-ladwein-Workspaces-${OLD_NAME}"
NEW_PROJ="home-ladwein-Workspaces-${NEW_NAME}"

echo "==> Workspace-Ordner-Symlink (Legacy-Pfad)"
if [[ ! -e "${OLD_PATH}" ]]; then
  ln -s "${NEW_NAME}" "${OLD_PATH}"
  echo "    ${OLD_PATH} -> ${NEW_NAME}"
else
  echo "    ${OLD_PATH} existiert bereits"
fi

echo "==> Cursor-Projektordner"
if [[ -d "${CURSOR_PROJ}/${NEW_PROJ}" && ! -e "${CURSOR_PROJ}/${OLD_PROJ}" ]]; then
  ln -s "${NEW_PROJ}" "${CURSOR_PROJ}/${OLD_PROJ}"
  echo "    ${OLD_PROJ} -> ${NEW_PROJ}"
else
  echo "    Cursor-Projekt: bereits verknüpft oder fehlt"
fi

echo ""
echo "Fertig. Cursor starten und öffnen:"
echo "  ${OLD_WS}"
echo ""
echo "Agent-Transkripte:"
echo "  ${CURSOR_PROJ}/${NEW_PROJ}/agent-transcripts/"
