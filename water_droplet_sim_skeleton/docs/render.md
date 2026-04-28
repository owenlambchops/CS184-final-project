先给结论：**你负责的渲染模块，baseline 不该是“做一个通用 fluid renderer”，而该是“把 solver 每帧输出的水滴表面，稳定地渲染成像水的透明折射体”**。按你们的 proposal，必须项里本来就有 real-time refractive shading、zero gravity / vertical glass 交互场景；caustics、更多复杂表面是后续扩展，不该抢占你现在的主线。

更具体地说，我建议你走 **mesh-based refraction pipeline**，不要把 Green 那套粒子流体 screen-space surface reconstruction 当成 baseline 直接照搬。原因是你们仿真主论文本身就是 **deformable surface model**：它把小水滴建模成边界表面，用 mean curvature flow、contact angle 和 mesh operators 去推进，而不是先给你一堆粒子。Green 的经典流程是为粒子流体准备的：先生成粒子深度图，再平滑深度，再从深度重建法线和位置，最后着色。你们已经有显式水滴表面 mesh，所以工程上更短的路径是：**直接渲染水滴 mesh 到离屏缓冲，然后做折射/菲涅耳合成**。这是基于两类方法的工程判断。([College of Engineering][1])

你现在真正要做的，建议拆成 5 件事。

1. **搭好 render graph 和 FBO。**
   你至少需要一个 `scene pass` 和一个 `droplet pass`。`scene pass` 先渲染所有非水滴内容，比如背景、玻璃、承载平面/曲面，输出 `sceneColor` 和 `sceneDepth`；`droplet pass` 再把水滴渲染到单独的离屏纹理里，至少输出法线和深度，最好再留一个 thickness 通道。Khronos 的文档里对 FBO 的定义很清楚：用户自建 framebuffer 的 attachment 可以是纹理或 renderbuffer，本质上就是“离屏渲染目标”；GPU Gems 的通用折射做法也是先把非折射场景渲染成一张纹理，再在下一 pass 中采样它。([Khronos Wiki][2])

2. **先做“静态水滴 mesh 的折射”，再接 solver。**
   不要一开始就和 simulation 同步调试。先拿一个固定半球/球冠 mesh，证明你的 renderer 能输出“像水”的结果。你这一步的目标只有三个：能读 `sceneColor`，能得到水滴表面的 view-space normal，能正确写入和比较 depth。这样做的好处是，你可以把渲染问题和仿真问题彻底隔离。Green 的资料把整体思路说得很直白：表面着色需要位置、法线、深度和与场景的正确深度合并；你们只是跳过了前面的“从粒子深度重建表面”那一步。

3. **核心 shader：refraction + Fresnel + specular。**
   折射最实用的实时做法不是物理追踪，而是 **用法线去扰动背景纹理坐标**。GPU Gems 2 的通用折射章节给的就是这条路线：先渲染非折射场景到纹理 `S`，再在折射物体 pass 中用法线贴图/法线方向对采样坐标做小位移。你们这里没有 normal map，而是直接用水滴 mesh 的表面法线即可。反射部分用 Fresnel 混合，最省事的是 Schlick 近似：`F = F0 + (1 - F0)(1 - dot(N, V))^5`。NVIDIA 的 Schlick 章节给出了这个公式，并指出水的折射率大约是 `η = 1.333`；用 `R0 = ((η1 - η2)/(η1 + η2))^2` 算，空气-水边界的 `F0` 大约是 `0.0204`。所以你第一版完全可以写成：
   `final = (1 - F) * refractedColor + F * reflectedColor + specular`。([NVIDIA Developer][3])

4. **加 thickness；这是“像水”与“不像水”的分水岭。**
   Green 的 slides 明确指出：只渲染离摄像机最近的那一层表面时，透明流体看起来会怪，因为你看不到前表面后面的液体层；一个有效补救是用 **thickness through volume** 去做颜色衰减。你们不是粒子流体，但这个思想完全可以借过来。最适合 mesh 水滴的做法，是渲染一对 front/back depth，然后把两者差值近似为 thickness；Imai 那篇更进一步，把 front-facing 和 back-facing surface 的 depth maps 成对生成，用多层 refraction 和 Beer–Lambert attenuation 提高真实感。你们 baseline 不需要做到四层折射，但**front/back depth -> thickness -> attenuation** 这条线非常值得借鉴。([NVIDIA Developer Download][4])

5. **边界质量和性能优化最后做，不要最先做。**
   如果你们后面发现 thickness 或 depth 在轮廓附近很脏，不要先上重武器。Green 先用 bilateral filter 保护轮廓；Truong 和 Yuksel 后来专门指出，screen-space fluid 的常见问题就是深度边界附近会被错误平滑，他们提出 narrow-range filter 来在不连续附近保边界，作者页面还直接给了 paper 和 code 链接。对你们这种显式 mesh 场景，我的建议是：**优先只滤 thickness 或 normal，不要盲目把主 depth 全局模糊**。另外，Green 也明确提到 thickness pass 很吃 fill-rate，可以降分辨率。

---

## 你这个模块的最小交付物

如果按我们之前的 skeleton，你主要对应这些文件：

`render/refractive_renderer.h/.cpp`
负责整个 pass 流程：创建 FBO、管理 `sceneColor / sceneDepth / dropletNormal / dropletDepth / dropletThickness`，以及最终 composite。

`render/gpu_mesh_buffer.h/.cpp`
负责把 solver 给的顶点位置、法线上传到 GPU，并绘制 indexed mesh。

`render/droplet_gpu_cache.h/.cpp`
负责管理多个 droplet 的 GPU buffer，同步 CPU-side `Droplet` 数据。

`assets/shaders/scene.vert/.frag`
渲染背景、玻璃、平面或 height field。这里先做简单光照就行，别在这一步卷材质。

`assets/shaders/droplet_gbuffer.vert/.frag`
把水滴 mesh 画进离屏纹理。建议第一版输出：

- view-space normal
- linear depth 或 eye-space depth
- 可选 thickness/front-back depth

`assets/shaders/refract_composite.vert/.frag`
最终合成 pass。读 `sceneColor`、`sceneDepth`、`dropletNormal`、`dropletThickness`，做折射偏移、Fresnel、reflection/specular、透明度/吸收。

---

## 我建议你的实现顺序

第一步，**把 static droplet 做出来**。
固定一个测试 mesh，在平面上渲染，看起来要像玻璃珠/水滴，而不是磨砂塑料。

第二步，**把 refraction 跑通**。
先不做 thickness，只做 `sceneColor` 采样偏移 + Schlick Fresnel。只要这一步成立，视觉上就已经有 70% 了。GPU Gems 这条路线本来就是为高效实时折射设计的。([NVIDIA Developer][3])

第三步，**补 thickness**。
这一步会明显提升水滴的体感。可以先用近似 thickness，再升级成 front/back depth 差。Green 的 thickness shading 和 Beer’s law 足够作为第一版参考；Imai 则是你后面想升级多层折射时再看。([NVIDIA Developer Download][4])

第四步，**接 solver**。
让渲染只依赖“每帧顶点位置 + 法线 + face index”，不要让渲染知道 mean curvature flow、volume correction 之类的物理细节。

第五步，**加 debug view**。
强烈建议你做 4 个调试开关：

- sceneColor
- droplet normal
- droplet depth
- droplet thickness
  不然你后面会非常难 debug。

---

## 有哪些文献最值得看

最先读这 5 个，足够你开工：

**1. GPU Gems 2, Chapter 19: Generic Refraction Simulation**
这是你最直接的 baseline 参考。它讲的就是：先渲染非折射场景，再通过法线扰动采样坐标做折射。你现在最该先把这章读明白。([NVIDIA Developer][3])

**2. Simon Green, Screen Space Fluid Rendering for Games**
虽然它是粒子流体路线，但里面关于 depth、bilateral filter、Fresnel、thickness、caustics 的组织方式非常实用，适合你拿来设计 pass。重点看 overview、bilateral、Fresnel、thickness 那几页。

**3. A Deformable Surface Model for Real-Time Water Drop Animation**
这篇不是渲染论文，但它决定了你的输入数据长什么样：你拿到的是 surface mesh，不是粒子云。这会直接影响你的 renderer 该怎么简化。([College of Engineering][1])

**4. Real-Time Screen-Space Liquid Rendering with Complex Refractions**
只有在你第一版已经做完之后再看。它的价值在于：告诉你 thickness/front-back depth 和多层 refraction 怎么往上升级；但别把它当 week 1 目标。

**5. The Schlick Fresnel Approximation**
这是你写 final composite shader 时最应该顺手放进代码里的公式来源。水的 IOR、Schlick 公式、`R0` 计算都在里面。

如果你后面只在一个问题上卡住——“为什么边界一糊就穿帮”——再去看 **A Narrow-Range Filter for Screen-Space Fluid Rendering**。它就是专门修这个痛点的，而且作者页面有 code。([TTNghia][5])

---

## 你现在别先做什么

**别先做 caustics。**
Green 的 slides 里把 caustics 单独列成较贵的 image-space 技术，而且只投到接收平面就已经不便宜了。你们 proposal 里 caustics 本来也只是 aspiration，不是 baseline。先把 refraction、Fresnel、thickness 做稳。 ([NVIDIA Developer Download][4])

**别先做多层 refraction。**
Imai 的四层深度图方案是更高级的效果线，不适合你一上来就碰。你们这个项目先把单次折射做得稳定、干净、可调，就已经很好了。

你现在最需要的不是更多论文，而是一个明确的 TODO。最硬的一版可以直接写成：

1. `scene pass` 输出 `sceneColor + sceneDepth`
2. `droplet pass` 输出 `normal + depth`
3. `composite pass` 做 `refraction + Fresnel + specular`
4. 增加 `thickness`
5. 增加 debug views
6. 最后再考虑 filter 和 polish

下一条我直接给你一版 **渲染模块的 TODO 清单 + `refract_composite.frag` 的 GLSL 骨架**。

[1]: https://www.cse.msu.edu/~ytong/Zhang2011.pdf "untitled"
[2]: https://wikis.khronos.org/opengl/Framebuffer "Framebuffer - OpenGL Wiki"
[3]: https://developer.nvidia.com/gpugems/gpugems2/part-ii-shading-lighting-and-shadows/chapter-19-generic-refraction-simulation "Chapter 19. Generic Refraction Simulation | NVIDIA Developer"
[4]: https://developer.download.nvidia.com/presentations/2010/gdc/Direct3D_Effects.pdf "Slide 1"
[5]: https://ttnghia.github.io/posts/narrow-range-filter/ "A Narrow-Range Filter for Screen-Space Fluid Rendering – Nghia Truong"
