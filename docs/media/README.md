# Demo media

把演示的截图 / GIF / 短视频放这里，供 README 引用。原始录屏（`.webm`/`.mkv`）不入库。

建议命名：

| 文件 | 内容 |
|---|---|
| `01_nav2.gif` | Nav2 把车开进 A 巷（终端 + Gazebo） |
| `02_vln.gif` | 一条自然语言指令 → 9 段航迹 → 到达 B-L-07-2 |
| `03_seg2_capture.gif` | 车在每个 bay 转 ±90° 拍照 |
| `warehouse_demo.gif` | 当前这段通用仓库行驶预览（8 s, 5.2 MB） |
| `warehouse_demo_720p.mp4` | 同一段完整 78 s 录屏（720p, 4.4 MB） |

约定：

* GIF ≤ 10 MB、8–15 秒、宽 800–1280 px；太长就放 YouTube / Bilibili 外链。
* 原始 `.webm`/`.mkv` 已在 `.gitignore` 里：提交前用 ffmpeg 压成 MP4/GIF。
* 截图用 1920×1080 PNG，命名 `01_nav2.png` 这种。
* GitHub 单文件硬上限 100 MB（>50 MB 会警告），不要把长视频直接提交。

在 README 里引用：

```markdown
![Nav2 demo](docs/media/01_nav2.gif)
```

三条演示的具体命令见 [docs/DEMO_GUI.md](../DEMO_GUI.md)。

## 压缩命令（参考）

```bash
# 2560x1600 VP8 WebM -> 1280x800 H.264 MP4
ffmpeg -y -i input.webm   -vf "fps=15,scale=1280:-2:flags=lanczos"   -c:v libx264 -preset medium -crf 30 -pix_fmt yuv420p -an   -movflags +faststart output.mp4

# 8 秒 GIF 预览（800 px, 8 fps, 96 色）
ffmpeg -y -i input.webm -ss 15 -t 8   -vf "fps=8,scale=800:-2:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=96[p];[s1][p]paletteuse=dither=bayer:bayer_scale=5"   -loop 0 output.gif
```
