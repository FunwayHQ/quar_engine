#!/usr/bin/env python3
"""
Simple HTTPS server for mobile testing.
Generates a self-signed certificate automatically.
"""

import http.server
import ssl
import os
import subprocess
import sys

PORT = 8443
CERT_FILE = '.dev-cert.pem'
KEY_FILE = '.dev-key.pem'

def generate_cert():
    """Generate a self-signed certificate for local development."""
    if os.path.exists(CERT_FILE) and os.path.exists(KEY_FILE):
        print(f"Using existing certificate: {CERT_FILE}")
        return

    print("Generating self-signed certificate...")

    # Get local IP for the certificate
    import socket
    hostname = socket.gethostname()
    local_ip = socket.gethostbyname(hostname)

    cmd = [
        'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
        '-keyout', KEY_FILE,
        '-out', CERT_FILE,
        '-days', '365',
        '-nodes',
        '-subj', f'/CN=localhost',
        '-addext', f'subjectAltName=DNS:localhost,IP:127.0.0.1,IP:{local_ip}'
    ]

    try:
        subprocess.run(cmd, check=True, capture_output=True)
        print(f"Certificate generated for localhost and {local_ip}")
    except subprocess.CalledProcessError as e:
        print(f"Error generating certificate: {e}")
        print("Trying simpler command...")
        # Fallback for older openssl versions
        cmd = [
            'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
            '-keyout', KEY_FILE,
            '-out', CERT_FILE,
            '-days', '365',
            '-nodes',
            '-subj', '/CN=localhost'
        ]
        subprocess.run(cmd, check=True)
        print("Certificate generated (localhost only)")

def get_local_ip():
    """Get the local IP address."""
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('8.8.8.8', 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def main():
    generate_cert()

    local_ip = get_local_ip()

    # Create SSL context
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(CERT_FILE, KEY_FILE)

    # Create and configure server
    handler = http.server.SimpleHTTPRequestHandler
    server = http.server.HTTPServer(('0.0.0.0', PORT), handler)
    server.socket = context.wrap_socket(server.socket, server_side=True)

    print(f"""
╔══════════════════════════════════════════════════════════════╗
║                    QUAR Engine HTTPS Server                   ║
╠══════════════════════════════════════════════════════════════╣
║                                                              ║
║  Feature Demo:  https://localhost:{PORT}/sdk/index.html      ║
║  AR Cube Demo:  https://localhost:{PORT}/sdk/ar-demo.html    ║
║                                                              ║
║  Network: https://{local_ip}:{PORT}/sdk/ar-demo.html         ║
║                                                              ║
║  ⚠️  On mobile, you'll see a certificate warning.            ║
║      Tap "Advanced" → "Proceed" to continue.                 ║
║                                                              ║
╚══════════════════════════════════════════════════════════════╝
    """)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nServer stopped.")

if __name__ == '__main__':
    main()
