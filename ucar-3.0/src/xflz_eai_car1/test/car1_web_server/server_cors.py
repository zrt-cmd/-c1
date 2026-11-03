"""
带 CORS 头的简易 HTTP 服务器 —— 解决 Trae IDE preview 把 XHR 当跨源请求的问题
用法：python server_cors.py [port]
默认 8080
"""
import http.server
import socketserver
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080

class CORSRequestHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        # 允许任意 origin（本地开发用，足够安全）
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', '*')
        # 防止 STLLoader 内部对 .STL 这种二进制文件的奇怪编码
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def log_message(self, format, *args):
        # 输出更简洁
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), format % args))

if __name__ == '__main__':
    with socketserver.TCPServer(('0.0.0.0', PORT), CORSRequestHandler) as httpd:
        print(f"CORS-enabled server running on port {PORT}")
        print(f"  Local:   http://localhost:{PORT}/viewer/")
        print(f"  Network: http://<your-ip>:{PORT}/viewer/")
        print(f"  (Use 'ipconfig' to find your IP address)")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nStopped.")
