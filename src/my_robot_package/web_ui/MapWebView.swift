import SwiftUI
import WebKit

struct MapWebView: UIViewRepresentable {
    let ipAddress: String
    
    func makeUIView(context: Context) -> WKWebView {
        let webView = WKWebView()
        // Makes the webview background match the dark theme
        webView.isOpaque = false
        webView.backgroundColor = .clear
        webView.scrollView.backgroundColor = .clear
        return webView
    }
    
    func updateUIView(_ webView: WKWebView, context: Context) {
        // We point to your existing Flask server
        if let url = URL(string: "http://\(ipAddress):8080") {
            let request = URLRequest(url: url)
            webView.load(request)
        }
    }
}