# Robot Credential Record Template

This is a template only. Never enter real passwords, API keys, Wi-Fi passwords,
recovery codes, or private SSH keys in the Git-tracked copy of this file.

Store the completed record in a password manager. If a printable recovery copy
is needed, keep it offline with the robot hardware. A local file named
`docs/CREDENTIALS.private.md` is ignored by this repository, but a password
manager is preferable because an ignored plaintext file can still be lost or
copied accidentally.

## GitHub

- Repository: `https://github.com/harshcshah97-collab/mapping-robot`
- Repository owner/account: `________________________________`
- Account email: `________________________________`
- Authentication method: SSH key / passkey / token / other: `______________`
- Recovery-code storage location: `________________________________`
- Two-factor device/recovery owner: `________________________________`

GitHub account passwords and tokens belong in the password manager, not here.

## Raspberry Pi

- Robot name: `Bob`
- Preferred hostname: `harsh-rpi.local` (last-known; verify at recommissioning)
- Linux username: `harsh`
- SSH authentication: public-key / password fallback: `____________________`
- Password-manager item name: `________________________________`
- Recovery media or console method: `________________________________`
- microSD/SSD device label and serial suffix: `________________________________`

Last-known addresses seen in the archive workstation's SSH records:

- `10.21.10.17`
- `192.168.3.2`
- `192.168.1.216`
- `192.168.1.68`

These are discovery hints, not permanent assignments. DHCP can change them and
some may refer to previous networks.

## Robot network

- Intended private SSID: `________________________________`
- Wi-Fi password-manager item: `________________________________`
- Router/admin record: `________________________________`
- Reserved IP, if configured: `________________________________`
- VLAN/firewall/VPN notes: `________________________________`

The web UI, rosbridge, and BLE provisioning interface currently have no client
authentication. They must not be exposed to the public internet.

## OpenAI assistant

- OpenAI project/account: `________________________________`
- API-key password-manager item: `________________________________`
- Billing/usage-limit owner: `________________________________`
- Expected model override, if any: `________________________________`
- Robot secret file: `/home/harsh/.config/mapping-robot/assistant.env`
- Required mode: `0600`, owner `harsh`

The file format on the robot is:

```dotenv
OPENAI_API_KEY=retrieve-from-password-manager
# Optional:
# OPENAI_MODEL=gpt-4o
# ROBOT_MIC_SAMPLE_RATE=16000
```

## Physical recovery

- Battery model/serial suffix: `________________________________`
- Charger location: `________________________________`
- Physical motor disconnect/E-stop location: `________________________________`
- Wiring-label legend location: `________________________________`
- Spare SD/SSD image location and date: `________________________________`
- Encrypted backup location and decryption owner: `________________________________`
- Last known-good Git tag/commit: `________________________________`

## Final archive check

- [ ] GitHub access tested from a second device.
- [ ] SSH public key and fallback console method tested.
- [ ] Password-manager record shared with the intended future operator.
- [ ] OpenAI key is absent from Git and shell-history exports.
- [ ] Network/UI exposure is limited to a trusted private network or VPN.
- [ ] Encrypted local-state backup can be decrypted and lists its contents.
