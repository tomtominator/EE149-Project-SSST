#include <WebServer.h>
#include <WiFi.h>
#include <esp32cam.h>
 
const char* WIFI_SSID = "freewifipasswordispassword";
const char* WIFI_PASS = "pokemon4950";
 
WebServer server(80);
 
static auto Res = esp32cam::Resolution::find(800, 600);
void serveJpg()
{
  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("CAPTURE FAIL");
    server.send(503, "", "");
    return;
  }
  Serial.printf("CAPTURE OK %dx%d %db\n", frame->getWidth(), frame->getHeight(),
                static_cast<int>(frame->size()));
  server.setContentLength(frame->size());
  server.send(200, "image/jpeg");
  WiFiClient client = server.client();
  frame->writeTo(client);
}
 

 
void handleRequest()
{
  if (!esp32cam::Camera.changeResolution(hiRes)) {
    Serial.println("fail");
  }
  serveJpg();
}
 

 
void  setup(){
  Serial.begin(115200);
  Serial.println();
  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    cfg.setResolution(Res);
    cfg.setBufferCount(2);
    cfg.setJpeg(80);
    bool ok = Camera.begin(cfg);
  }
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("http://");
  Serial.println(WiFi.localIP());
  server.on("/image", handleRequest);
  
 
  server.begin();
}
 
void loop()
{
  server.handleClient();
}