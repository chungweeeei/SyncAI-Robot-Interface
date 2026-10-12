# A WiFi network NetworkManager has saved on the robot (a connection profile),
# as listed by ListWifiConnections.
string uuid
# The profile's name; usually the SSID, but nmcli lets the two differ.
string name
string ssid
# True for the profile that is up on the WiFi device right now.
bool active
# Unix seconds of the last successful activation, 0 if never.
int64 last_used
