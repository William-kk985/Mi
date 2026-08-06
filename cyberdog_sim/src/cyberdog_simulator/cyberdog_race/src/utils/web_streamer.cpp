#include "cyberdog_race/utils/web_streamer.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstring>
#include <sstream>

// ═══════════════════════════════════════════════════════════
// HTML 交互式双面板页面
//   · 左面板固定原始画面  · 右面板 Tab 切换标注/雷达
//   · [+] 放大面板 / [-] 还原
// ═══════════════════════════════════════════════════════════
static const char* kHtmlPage = R"raw(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>CyberDog 调试面板</title>
<style>
  :root { --bg:#0d1117; --panel:#161b22; --border:#30363d; --accent:#e94560;
          --green:#3fb950; --blue:#58a6ff; --text:#c9d1d9; }
  * { margin:0; padding:0; box-sizing:border-box; }
  body { background:var(--bg); color:var(--text); font-family:'Segoe UI',sans-serif;
         padding:8px; min-height:100vh; }
  .topbar { display:flex; gap:16px; align-items:center; padding:6px 12px;
            background:var(--panel); border:1px solid var(--border); border-radius:6px;
            margin-bottom:8px; font-size:0.82em; flex-wrap:wrap; }
  .topbar .st { color:var(--accent); font-weight:bold; }
  .main { display:flex; gap:8px; height:calc(100vh - 60px); transition:all 0.3s; }
  .panel { flex:1; background:var(--panel); border:1px solid var(--border);
           border-radius:8px; overflow:hidden; display:flex; flex-direction:column;
           transition:flex 0.35s ease; min-width:0; }
  .panel.full { flex:100; }
  .panel.mini { flex:0.06; min-width:40px; }
  .panel.mini .panel-body, .panel.mini .fps-bar { display:none; }
  .panel.mini .panel-hdr { writing-mode:vertical-lr; text-orientation:mixed; }
  .panel.mini .panel-hdr .hdr-right { display:none; }
  .panel-hdr { display:flex; align-items:center; justify-content:space-between;
               padding:6px 10px; background:#1c2333; border-bottom:1px solid var(--border);
               flex-shrink:0; min-height:34px; }
  .panel-hdr .hdr-title { font-size:0.85em; font-weight:600; white-space:nowrap; }
  .panel-hdr .hdr-right { display:flex; align-items:center; gap:6px; }
  .tab { background:none; border:1px solid transparent; color:#8b949e;
         padding:3px 8px; border-radius:4px; cursor:pointer; font-size:0.78em;
         white-space:nowrap; transition:all 0.15s; }
  .tab:hover { color:var(--text); border-color:var(--border); }
  .tab.on { color:var(--green); border-color:var(--green); background:#1a3020; }
  .btn-exp { background:none; border:1px solid var(--border); color:#8b949e;
             cursor:pointer; font-size:0.85em; padding:2px 7px; border-radius:4px;
             line-height:1; transition:all 0.15s; }
  .btn-exp:hover { color:var(--text); border-color:var(--accent); }
  .panel-body { flex:1; overflow:hidden; position:relative; background:#000; }
  .panel-body img { width:100%; height:100%; object-fit:contain; display:block; }
  .fps-bar { font-size:0.72em; padding:3px 10px; background:#0d1b36;
             color:var(--green); border-top:1px solid var(--border); flex-shrink:0; }
  .placeholder { display:flex; align-items:center; justify-content:center;
                 height:100%; color:#30363d; font-size:1.1em; }
</style>
</head>
<body>
<div class="topbar" id="topbar">
  <span>🐕 CyberDog</span>
  <a href="/settings" style="color:#58a6ff;text-decoration:none;font-size:0.85em">⚙️</a>
  <span>赛道:<span class="st" id="st-stage">-</span></span>
  <span>yaw:<span id="st-yaw">-</span></span>
  <span>odom:(<span id="st-ox">-</span>,<span id="st-oy">-</span>)</span>
  <span>身高:<span id="st-h">-</span>m</span>
  <span>TOF:<span id="st-tof">-</span>m</span>
  <span>超声:<span id="st-ultra">-</span>m</span>
  <span style="color:#8b949e;margin-left:auto" id="st-time">--:--:--</span>
</div>
<div class="main" id="main">
  <!-- 左面板：原始画面 -->
  <div class="panel" id="pnl-left">
    <div class="panel-hdr">
      <span class="hdr-title">📷 原始画面</span>
      <span class="hdr-right">
        <button class="btn-exp" onclick="togglePanel('left')" title="放大/还原">⛶</button>
      </span>
    </div>
    <div class="panel-body"><img id="img-raw" src="/stream"></div>
    <div class="fps-bar" id="fps-raw">等待…</div>
  </div>
  <!-- 右面板：标注 / 雷达 -->
  <div class="panel" id="pnl-right">
    <div class="panel-hdr">
      <span class="hdr-title" id="right-title">🔍 标注画面</span>
      <span class="hdr-right">
        <select class="stream-sel" id="stream-sel" onchange="switchStream()">
          <option value="debug" selected>🔍 标注画面</option>
          <option value="lidar">📡 LiDAR 雷达</option>
          <option value="dark">🌑 暗图</option>
          <option value="track">🗺️ 里程轨迹</option>
          <option value="telem">📊 遥测仪表</option>
          <option value="d435">🔆 D430i左红外</option>
          <option value="infra2">🔆 D430i右红外</option>
          <option value="depth">🌊 深度图</option>
        </select>
        <button class="btn-exp" onclick="togglePanel('right')" title="放大/还原">⛶</button>
      </span>
    </div>
    <div class="panel-body"><img id="img-right" src="/stream/debug"></div>
    <div class="fps-bar" id="fps-right">等待…</div>
  </div>
</div>
<script>
const streams={debug:'/stream/debug',lidar:'/stream/lidar',dark:'/stream/dark',
  track:'/stream/track',telem:'/stream/telemetry',d435:'/stream/d435',
  depth:'/stream/depth',infra2:'/stream/infra2'};
  function switchStream(){
  const sel=document.getElementById('stream-sel');
  const v=sel.value, opt=sel.options[sel.selectedIndex];
  document.getElementById('right-title').textContent=opt.text;
  document.getElementById('img-right').src=streams[v]+'?t='+Date.now();
  resetFPS('right');
}
let expanded=null;
function togglePanel(side){
  const L=document.getElementById('pnl-left'),R=document.getElementById('pnl-right');
  if(expanded===side){L.className='panel';R.className='panel';expanded=null;}
  else{if(side==='left'){L.className='panel full';R.className='panel mini';}
       else{R.className='panel full';L.className='panel mini';}
       expanded=side;}
}
const fps={raw:{last:0,frames:0},right:{last:0,frames:0}};
function resetFPS(k){fps[k].last=0;fps[k].frames=0;}
function trackFPS(imgId,labelId,key){
  const el=document.getElementById(labelId);
  document.getElementById(imgId).addEventListener('load',()=>{
    const f=fps[key];f.frames++;
    const now=performance.now();
    if(now-f.last>=1000){
      el.textContent='FPS:'+f.frames+' | ~'+
        ((now-f.last)/f.frames).toFixed(0)+'ms';
      f.frames=0;f.last=now;
    }
  });
}
trackFPS('img-raw','fps-raw','raw');
trackFPS('img-right','fps-right','right');
// 顶部时钟
setInterval(()=>{document.getElementById('st-time').textContent=
  new Date().toLocaleTimeString();},1000);
// 遥测轮询（占位，后续接 /api/telemetry）
setInterval(()=>{fetch('/api/telemetry').then(r=>r.json()).then(d=>{
  if(d.stage)document.getElementById('st-stage').textContent=d.stage;
  if(d.yaw!=null)document.getElementById('st-yaw').textContent=d.yaw.toFixed(2);
  if(d.ox!=null)document.getElementById('st-ox').textContent=d.ox.toFixed(2);
  if(d.oy!=null)document.getElementById('st-oy').textContent=d.oy.toFixed(2);
  if(d.height!=null)document.getElementById('st-h').textContent=d.height.toFixed(2);
  if(d.tof!=null)document.getElementById('st-tof').textContent=d.tof.toFixed(2);
  if(d.ultra!=null)document.getElementById('st-ultra').textContent=d.ultra.toFixed(2);
}).catch(()=>{});},500);
</script>
</body>
</html>
)raw";

// ── 相机设置页面 ──
static const char* kSettingsPage = R"raw(
<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>相机设置</title>
<style>
  :root{--bg:#0d1117;--panel:#161b22;--border:#30363d;--accent:#e94560;--green:#3fb950;--blue:#58a6ff;--text:#c9d1d9}
  *{margin:0;padding:0;box-sizing:border-box}
  body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;padding:20px;max-width:500px;margin:0 auto}
  h1{color:var(--accent);font-size:1.2em;margin-bottom:4px}
  a.back{color:var(--blue);text-decoration:none;font-size:0.82em}
  .row{margin:16px 0}.row label{display:block;font-size:0.85em;color:#8b949e;margin-bottom:4px}
  .row label span{float:right;color:var(--green);font-size:0.8em}
  input[type=range]{width:100%;accent-color:var(--accent)}
  .btn{background:var(--accent);color:#fff;border:none;padding:8px 20px;border-radius:6px;cursor:pointer;font-size:0.85em;margin-right:8px}
  .btn.reset{background:var(--border)}.msg{font-size:0.78em;color:var(--green);margin-top:8px}
  .info{font-size:0.75em;color:#555;margin-top:20px;line-height:1.6}
</style>
</head>
<body>
<h1>⚙️ 相机设置</h1>
<a class="back" href="/">← 返回监控</a>
<div class="row">
  <label>曝光偏移（降暗） <span id="v-eo">-30</span></label>
  <input type="range" id="eo" min="-100" max="0" value="-30" oninput="up('eo','v-eo')">
  <div style="font-size:0.7em;color:#555;margin-top:2px">-100=最暗 &nbsp; 0=原画</div>
</div>
<div class="row">
  <label>🌑暗图 JPEG 质量 <span id="v-q">70</span></label>
  <input type="range" id="q" min="10" max="100" value="70" oninput="up('q','v-q')">
</div>
<div style="margin-top:16px">
  <button class="btn" onclick="applyAll()">💾 应用并保存</button>
  <button class="btn reset" onclick="resetAll()">↩ 默认</button>
</div>
<div class="msg" id="msg"></div>
<div class="info">
  💡 原始/标注流保持最高质量，JPEG 质量只影响 🌑暗图。<br>
  💡 原始画面始终显示相机直出（高曝光亮图）。<br>
  💡 右面板下拉选「🌑 暗图」即可观察降暗效果。<br>
  💡 视觉算法使用原始图像，不受曝光偏移影响。<br>
  💡 点击「💾 应用并保存」写入 camera_config.conf，重启自动加载。
</div>
<script>
function up(s,v){document.getElementById(v).textContent=document.getElementById(s).value}
async function applyAll(){
  const eo=document.getElementById('eo').value;
  const q=document.getElementById('q').value;
  await fetch('/api/camera/settings?exposure_offset='+eo+'&jpeg_quality='+q);
  document.getElementById('msg').textContent='✅ 已保存到 camera_config.conf';
}
async function resetAll(){
  document.getElementById('eo').value=-30;up('eo','v-eo');
  document.getElementById('q').value=70;up('q','v-q');
  await applyAll();
}
(async()=>{
  try{const r=await fetch('/api/camera/settings');const d=await r.json();
    document.getElementById('eo').value=d.exposure_offset||-30;up('eo','v-eo');
    document.getElementById('q').value=d.jpeg_quality||70;up('q','v-q');
  }catch(e){}
})();
</script>
</body>
</html>
)raw";

// ═══════════════════════════════════════════════════════════
// 底层 I/O 辅助
// ═══════════════════════════════════════════════════════════

static std::string read_line(int fd) {
    std::string line;
    char c;
    int count = 0;
    while (count < 4096 && read(fd, &c, 1) == 1) {  // P7: 防慢速/恶意客户端拖死 accept
        count++;
        if (c == '\r') continue;
        if (c == '\n') break;
        line += c;
    }
    return line;
}

static void send_header(int fd, int code, const char* status,
                        const char* content_type, size_t content_len = 0) {
    char buf[512];
    int n = snprintf(buf, sizeof(buf),
                     "HTTP/1.1 %d %s\r\n"
                     "Content-Type: %s\r\n"
                     "Connection: close\r\n"
                     "Access-Control-Allow-Origin: *\r\n"
                     "Cache-Control: no-cache\r\n",
                     code, status, content_type);
    if (n > 0 && n < static_cast<int>(sizeof(buf))) write(fd, buf, n);  // P4: 防溢出
    if (content_len > 0) {
        n = snprintf(buf, sizeof(buf), "Content-Length: %zu\r\n", content_len);
        if (n > 0 && n < static_cast<int>(sizeof(buf))) write(fd, buf, n);
    }
    write(fd, "\r\n", 2);
}

static bool send_all(int fd, const void* data, size_t len) {
    const auto* p = static_cast<const char*>(data);
    size_t remaining = len;
    while (remaining > 0) {
        ssize_t n = write(fd, p, remaining);
        if (n <= 0) return false;
        p += n;
        remaining -= n;
    }
    return true;
}

/// 发送一帧 MJPEG multipart 段
static bool send_mjpeg_part(int fd, const std::vector<uint8_t>& jpeg) {
    char hdr[256];
    int hn = snprintf(hdr, sizeof(hdr),
                      "--CyberDogFrame\r\n"
                      "Content-Type: image/jpeg\r\n"
                      "Content-Length: %zu\r\n"
                      "\r\n",
                      jpeg.size());
    return send_all(fd, hdr, hn) &&
           send_all(fd, jpeg.data(), jpeg.size()) &&
           send_all(fd, "\r\n", 2);
}

// ═══════════════════════════════════════════════════════════
// 公开接口
// ═══════════════════════════════════════════════════════════

bool WebStreamer::start(int port, int max_clients) {
    if (running_.load()) return false;
    // ★ 忽略 SIGPIPE：浏览器断开连接后旧推流线程 write() 会触发 SIGPIPE，
    //   不忽略则整个进程崩溃（"切换画面就崩溃"的根因）。忽略后 write 返回 EPIPE，
    //   send_all() 返回 false → client_handler 正常退出。
    signal(SIGPIPE, SIG_IGN);
    max_clients_ = max_clients;
    running_ = true;
    load_settings();
    server_thread_ = std::thread(&WebStreamer::server_loop, this, port);
    return true;
}

void WebStreamer::stop() {
    running_ = false;
    frame_cv_.notify_all();

    // 强制唤醒阻塞在 accept() 的 server 线程
    if (server_fd_ >= 0) shutdown(server_fd_, SHUT_RDWR);
    if (server_thread_.joinable()) server_thread_.join();

    // 客户端线程已 detach：轮询等待计数归零（最多 5s），确保 this 安全析构
    for (int i = 0; i < 50 && active_clients_.load() > 0; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (active_clients_.load() > 0) {
        fprintf(stderr, "[WebStreamer] WARN: %d 客户端线程未在 5s 内退出\n",
                active_clients_.load());
    }
    fprintf(stderr, "[WebStreamer] stopped\n");
}

void WebStreamer::push_frame(const cv::Mat& frame) {
    if (!running_.load() || frame.empty()) return;

    std::vector<uint8_t> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    cv::imencode(".jpg", frame, jpeg, params);

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        jpeg_buffer_.swap(jpeg);
        has_frame_ = true;
        frame_seq_++;
    }
    frame_cv_.notify_all();
}

void WebStreamer::push_debug_frame(const cv::Mat& frame) {
    if (!running_.load() || frame.empty()) return;

    std::vector<uint8_t> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    cv::imencode(".jpg", frame, jpeg, params);

    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        jpeg_debug_buffer_.swap(jpeg);
        has_debug_frame_ = true;
        debug_frame_seq_++;
    }
    frame_cv_.notify_all();
}

void WebStreamer::push_lidar_frame(const cv::Mat& frame) {
    if (!running_.load() || frame.empty()) return;
    std::vector<uint8_t> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    cv::imencode(".jpg", frame, jpeg, params);
    { std::lock_guard<std::mutex> lock(frame_mutex_);
      jpeg_lidar_buffer_.swap(jpeg); has_lidar_frame_ = true; lidar_frame_seq_++; }
    frame_cv_.notify_all();
}

void WebStreamer::push_track_frame(const cv::Mat& frame) {
    if (!running_.load() || frame.empty()) return;
    std::vector<uint8_t> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    cv::imencode(".jpg", frame, jpeg, params);
    { std::lock_guard<std::mutex> lock(frame_mutex_);
      jpeg_track_buffer_.swap(jpeg); has_track_frame_ = true; track_frame_seq_++; }
    frame_cv_.notify_all();
}

void WebStreamer::push_telemetry_frame(const cv::Mat& frame) {
    if (!running_.load() || frame.empty()) return;
    std::vector<uint8_t> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    cv::imencode(".jpg", frame, jpeg, params);
    { std::lock_guard<std::mutex> lock(frame_mutex_);
      jpeg_telem_buffer_.swap(jpeg); has_telem_frame_ = true; telem_frame_seq_++; }
    frame_cv_.notify_all();
}

void WebStreamer::push_d435_frame(const cv::Mat& frame) {
    if (!running_.load() || frame.empty()) return;
    std::vector<uint8_t> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 65};
    cv::imencode(".jpg", frame, jpeg, params);
    { std::lock_guard<std::mutex> lock(frame_mutex_);
      jpeg_d435_buffer_.swap(jpeg); has_d435_frame_ = true; d435_frame_seq_++; }
    frame_cv_.notify_all();
}

// ── D430i 右目红外（mono8→灰度，2026-08-06 接入） ──
void WebStreamer::push_infra2_frame(const cv::Mat& frame) {
    if (!running_.load() || frame.empty()) return;
    std::vector<uint8_t> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 65};
    cv::imencode(".jpg", frame, jpeg, params);
    { std::lock_guard<std::mutex> lock(frame_mutex_);
      jpeg_infra2_buffer_.swap(jpeg); has_infra2_frame_ = true; infra2_frame_seq_++; }
    frame_cv_.notify_all();
}

void WebStreamer::push_dark_frame(const cv::Mat& frame) {
    if (!running_.load() || frame.empty()) return;
    std::vector<uint8_t> jpeg;
    int jq = jpeg_quality_;
    if (jq < 1) jq = 1;
    if (jq > 100) jq = 100;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jq};
    cv::imencode(".jpg", frame, jpeg, params);
    { std::lock_guard<std::mutex> lock(frame_mutex_);
      jpeg_dark_buffer_.swap(jpeg); has_dark_frame_ = true; dark_frame_seq_++; }
    frame_cv_.notify_all();
}

// ── 遥测数据更新（RaceController control_loop 调用，100Hz） ──
void WebStreamer::update_telemetry(float stage, float yaw, float ox, float oy, float height,
                                   float tof, float ultra) {
    std::lock_guard<std::mutex> lock(telemetry_mutex_);
    telemetry_.stage = stage;
    telemetry_.yaw = yaw;
    telemetry_.ox = ox;
    telemetry_.oy = oy;
    telemetry_.height = height;
    telemetry_.tof = tof;
    telemetry_.ultra = ultra;
}

// ── D430i 深度伪彩色帧（mono16→JET colormap 后） ──
void WebStreamer::push_depth_frame(const cv::Mat& frame) {
    if (!running_.load() || frame.empty()) return;
    std::vector<uint8_t> jpeg;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 70};
    cv::imencode(".jpg", frame, jpeg, params);
    { std::lock_guard<std::mutex> lock(frame_mutex_);
      jpeg_depth_buffer_.swap(jpeg); has_depth_frame_ = true; depth_frame_seq_++; }
    frame_cv_.notify_all();
}

// ═══════════════════════════════════════════════════════════
// accept 主循环
// ═══════════════════════════════════════════════════════════

void WebStreamer::server_loop(int port) {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) {
        perror("[WebStreamer] socket() failed");
        running_ = false;
        return;
    }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("[WebStreamer] bind() failed");
        close(server_fd_);
        server_fd_ = -1;
        running_ = false;
        return;
    }

    if (listen(server_fd_, 8) < 0) {
        perror("[WebStreamer] listen() failed");
        close(server_fd_);
        server_fd_ = -1;
        running_ = false;
        return;
    }

    fprintf(stderr, "[WebStreamer] HTTP server on 0.0.0.0:%d (max %d clients)\n",
            port, max_clients_);

    while (running_.load()) {
        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd_, reinterpret_cast<struct sockaddr*>(&client_addr),
                               &client_len);
        if (client_fd < 0) {
            if (running_.load()) perror("[WebStreamer] accept() failed");
            continue;
        }

        // P1: 用 atomic 计数替代遍历 zombie 线程向量
        if (active_clients_.load() >= max_clients_) {
            const char* busy = "Server busy, try later";
            send_header(client_fd, 503, "Service Unavailable", "text/plain", strlen(busy));
            send_all(client_fd, busy, strlen(busy));
            close(client_fd);
            continue;
        }

        // 设置超时
        struct timeval tv{1, 0};
        setsockopt(client_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

        // 解析请求行
        std::string request_line = read_line(client_fd);
        if (request_line.empty()) { close(client_fd); continue; }

        // 跳过其余头
        std::string hdr;
        while (running_.load()) { hdr = read_line(client_fd); if (hdr.empty()) break; }

        std::istringstream iss(request_line);
        std::string method, path, version;
        iss >> method >> path >> version;

        if (method != "GET") {
            send_header(client_fd, 405, "Method Not Allowed", "text/plain", 0);
            close(client_fd);
            continue;
        }

        // ★ 剥离 query string：浏览器切换流会带 ?t=时间戳 防缓存（如 /stream/debug?t=1712...）
        //   不剥离则路由匹配失败 → 404 → 切换黑屏（已 2026-08-06 定位）
        {
            size_t qpos = path.find('?');
            if (qpos != std::string::npos) path = path.substr(0, qpos);
        }

        // ── 路由：静态页面在主线程直接返回，流媒体 spawn 子线程 ──
        if (path == "/" || path == "/index.html") {
            std::string page(kHtmlPage);
            send_header(client_fd, 200, "OK", "text/html; charset=utf-8", page.size());
            send_all(client_fd, page.data(), page.size());
            close(client_fd);
        } else if (path == "/stream" || path == "/stream/debug" || path == "/stream/lidar" ||
                   path == "/stream/track" || path == "/stream/telemetry" || path == "/stream/d435" ||
                   path == "/stream/dark" ||
                   path == "/stream/depth" || path == "/stream/infra2") {
            active_clients_++;
            // detach：不存 vector（存了析构 joinable std::thread 会 std::terminate）
            // 退出靠 active_clients_ 计数归零 + stop() 轮询等待
            std::thread(&WebStreamer::client_handler, this, client_fd, path).detach();
        } else if (path == "/settings") {
            std::string page(kSettingsPage);
            send_header(client_fd, 200, "OK", "text/html; charset=utf-8", page.size());
            send_all(client_fd, page.data(), page.size());
            close(client_fd);
        } else if (path.find("/api/camera/settings") == 0) {
            auto qpos = path.find('?');
            if (qpos != std::string::npos) {
                std::string qs = path.substr(qpos + 1);
                auto get_val = [&](const char* key, int def) -> int {
                    std::string k(key); k += "=";
                    size_t p = qs.find(k);
                    if (p == std::string::npos) return def;
                    p += k.size();
                    size_t end = qs.find('&', p);
                    std::string v = qs.substr(p, end == std::string::npos ? std::string::npos : end - p);
                    try { return std::stoi(v); } catch (...) { return def; }
                };
                exposure_offset_ = get_val("exposure_offset", exposure_offset_);
                jpeg_quality_    = get_val("jpeg_quality", jpeg_quality_);
                save_settings();
            }
            char json[100];
            snprintf(json, sizeof(json),
                     "{\"exposure_offset\":%d,\"jpeg_quality\":%d}",
                     exposure_offset_, jpeg_quality_);
            send_header(client_fd, 200, "OK", "application/json", strlen(json));
            send_all(client_fd, json, strlen(json));
            close(client_fd);
        } else if (path == "/api/telemetry") {
            // 遥测 JSON（RaceController 通过 update_telemetry 填充）
            char json[256];
            float t_stage, t_yaw, t_ox, t_oy, t_h, t_tof, t_ultra;
            {
                std::lock_guard<std::mutex> lock(telemetry_mutex_);
                t_stage = telemetry_.stage; t_yaw = telemetry_.yaw;
                t_ox = telemetry_.ox; t_oy = telemetry_.oy;
                t_h = telemetry_.height; t_tof = telemetry_.tof; t_ultra = telemetry_.ultra;
            }
            int n = snprintf(json, sizeof(json),
                "{\"stage\":%.0f,\"yaw\":%.3f,\"ox\":%.3f,\"oy\":%.3f,\"height\":%.3f,\"tof\":%.3f,\"ultra\":%.3f}",
                t_stage, t_yaw, t_ox, t_oy, t_h, t_tof, t_ultra);
            send_header(client_fd, 200, "OK", "application/json", n);
            send_all(client_fd, json, n);
            close(client_fd);
        } else {
            const char* nf = "Not Found";
            send_header(client_fd, 404, "Not Found", "text/plain", strlen(nf));
            send_all(client_fd, nf, strlen(nf));
            close(client_fd);
        }
    }

    close(server_fd_);
    server_fd_ = -1;
}

// ═══════════════════════════════════════════════════════════
// 单客户端 MJPEG 流
// ═══════════════════════════════════════════════════════════

void WebStreamer::client_handler(int client_fd, const std::string& path) {
    // 流类型: 0=raw 1=debug 2=lidar 3=track 4=telem 5=d435_infra1 6=dark 9=depth 10=infra2
    int stype = 0;
    if      (path == "/stream/debug")         stype = 1;
    else if (path == "/stream/lidar")         stype = 2;
    else if (path == "/stream/track")         stype = 3;
    else if (path == "/stream/telemetry")     stype = 4;
    else if (path == "/stream/d435")          stype = 5;
    else if (path == "/stream/dark")          stype = 6;
    else if (path == "/stream/depth")         stype = 9;
    else if (path == "/stream/infra2")        stype = 10;

    // 发送 MJPEG HTTP 头
    const char* mjpeg_header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=--CyberDogFrame\r\n"
        "Connection: close\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache\r\n"
        "\r\n";
    if (!send_all(client_fd, mjpeg_header, strlen(mjpeg_header))) {
        active_clients_--;
        close(client_fd);
        return;
    }

    // 等待第一帧（最多 3 秒）
    uint64_t last_seq = 0;
    {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_cv_.wait_for(lock, std::chrono::seconds(3), [&] {
            if (!running_.load()) return true;
            switch (stype) {
                case 10: return has_infra2_frame_;
                case 9: return has_depth_frame_;
                case 6: return has_dark_frame_;
                case 5: return has_d435_frame_;
                case 4: return has_telem_frame_;
                case 3: return has_track_frame_;
                case 2: return has_lidar_frame_;
                case 1: return has_debug_frame_;
                default: return has_frame_;
            }
        });
        if (!running_.load()) {
            active_clients_--;
            close(client_fd);
            return;
        }
        switch (stype) {
            case 10: last_seq = infra2_frame_seq_; break;
            case 9: last_seq = depth_frame_seq_;  break;
            case 6: last_seq = dark_frame_seq_;  break;
            case 5: last_seq = d435_frame_seq_;  break;
            case 4: last_seq = telem_frame_seq_; break;
            case 3: last_seq = track_frame_seq_; break;
            case 2: last_seq = lidar_frame_seq_; break;
            case 1: last_seq = debug_frame_seq_; break;
            default: last_seq = frame_seq_;
        }
    }

    // 推流循环
    while (running_.load()) {
        // ★ 检测客户端断开：浏览器关闭连接后线程必须退出，否则线程堆积 +
        //   active_clients_ 占满上限 → 后续请求全 503 → 画面黑屏（2026-08-06 定位）
        {
            struct pollfd pfd;
            pfd.fd = client_fd;
            pfd.events = POLLIN;
            pfd.revents = 0;
            if (poll(&pfd, 1, 0) > 0) {
                if (pfd.revents & (POLLHUP | POLLERR)) break;      // 连接断开/异常
                if (pfd.revents & POLLIN) {                        // 对端有数据或关闭
                    char c;
                    if (recv(client_fd, &c, 1, MSG_PEEK | MSG_DONTWAIT) == 0) break;
                }
            }
        }
        std::vector<uint8_t> jpeg_copy;
        uint64_t current_seq;
        {
            std::unique_lock<std::mutex> lock(frame_mutex_);
            // 1s 短超时：快速回到 poll 检测断开（原来 5s 太慢导致断连线程堆积）
            bool got = frame_cv_.wait_for(lock, std::chrono::seconds(1), [&] {
                if (!running_.load()) return true;
                switch (stype) {
                    case 10: return infra2_frame_seq_ != last_seq;
                    case 9: return depth_frame_seq_  != last_seq;
                    case 6: return dark_frame_seq_  != last_seq;
                    case 5: return d435_frame_seq_  != last_seq;
                    case 4: return telem_frame_seq_ != last_seq;
                    case 3: return track_frame_seq_ != last_seq;
                    case 2: return lidar_frame_seq_ != last_seq;
                    case 1: return debug_frame_seq_ != last_seq;
                    default: return frame_seq_      != last_seq;
                }
            });
            if (!running_.load()) break;
            if (!got) continue;

            switch (stype) {
                case 10: jpeg_copy = jpeg_infra2_buffer_; current_seq = infra2_frame_seq_; break;
                case 9: jpeg_copy = jpeg_depth_buffer_; current_seq = depth_frame_seq_;  break;
                case 6: jpeg_copy = jpeg_dark_buffer_;  current_seq = dark_frame_seq_;  break;
                case 5: jpeg_copy = jpeg_d435_buffer_;  current_seq = d435_frame_seq_;  break;
                case 4: jpeg_copy = jpeg_telem_buffer_; current_seq = telem_frame_seq_; break;
                case 3: jpeg_copy = jpeg_track_buffer_; current_seq = track_frame_seq_; break;
                case 2: jpeg_copy = jpeg_lidar_buffer_; current_seq = lidar_frame_seq_; break;
                case 1: jpeg_copy = jpeg_debug_buffer_; current_seq = debug_frame_seq_; break;
                default: jpeg_copy = jpeg_buffer_;      current_seq = frame_seq_;
            }
            last_seq = current_seq;
        }

        if (jpeg_copy.empty()) continue;
        if (!send_mjpeg_part(client_fd, jpeg_copy)) break;
    }

    active_clients_--;
    close(client_fd);
}

// ═══════════════════════════════════════════════════════════
// 配置文件读写
// ═══════════════════════════════════════════════════════════

void WebStreamer::load_settings(const std::string& path) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return;
    char line[64];
    while (fgets(line, sizeof(line), f)) {
        int v;
        if (sscanf(line, "exposure_offset=%d", &v) == 1) exposure_offset_ = v;
        else if (sscanf(line, "jpeg_quality=%d", &v) == 1) jpeg_quality_ = v;
    }
    fclose(f);
}

void WebStreamer::save_settings(const std::string& path) {
    FILE* f = fopen(path.c_str(), "w");
    if (!f) return;
    fprintf(f, "exposure_offset=%d\njpeg_quality=%d\n",
            exposure_offset_, jpeg_quality_);
    fclose(f);
}
