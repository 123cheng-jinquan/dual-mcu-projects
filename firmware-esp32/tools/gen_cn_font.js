/* =============================================================================
 * 生成 LVGL 中文字库（lv_font_cn_NN.c）
 * -----------------------------------------------------------------------------
 * 为什么用 Node 脚本而不是直接敲命令行：
 *   Windows 的 PowerShell 在向子进程传"中文参数"时会走 ANSI 代码页，
 *   783 个汉字会被转成乱码（实测报错信息里就是一堆"瀹夋寜鎷"）。
 *   用 Node 读 UTF-8 文件、再以参数数组方式传给 lv_font_conv，可以完全绕开。
 *
 * 用法：
 *   cd D:\MyProject
 *   node firmware-esp32\tools\gen_cn_font.js [字号，默认 14]
 *
 * 前置：
 *   1. 把要显示的汉字都写进 firmware-esp32/tools/font_symbols.txt（UTF-8，无 BOM）
 *      —— 该文件由 tools/extract_symbols.ps1 自动从源码里扫出来
 *   2. npm install lv_font_conv（在 D:\MyProject 下，会装到 node_modules）
 *
 * ⚠ 只做 14px 一档：字库是"稀疏"的（--symbols 只打用到的字），
 *   783 字 × 4bpp ≈ 130KB，放 Flash 没问题。
 *   如果以后加了新的中文，**必须重新跑一遍**，否则新字会显示成方框。
 * ========================================================================== */
const fs = require('fs');
const path = require('path');
const { execFileSync } = require('child_process');

const size = parseInt(process.argv[2] || '14', 10);
/* bpp：每像素灰度位数。4bpp 抗锯齿最细腻但体积约为 2bpp 的两倍。
 * 本项目 783 个汉字：4bpp ≈ 520KB（偏大）、**2bpp ≈ 260KB**（够用，选它）。
 * 数据仪表界面用不上 16 级灰度，2bpp（4 级）在 14px 下看着完全正常。 */
const bpp = process.argv[3] || '2';
const root = path.resolve(__dirname, '..', '..');            // D:\MyProject
const symbolsFile = path.join(__dirname, 'font_symbols.txt');
const fontTtf = process.env.CN_FONT || 'C:\\Windows\\Fonts\\simhei.ttf';
const outFile = path.join(__dirname, '..', 'src', `lv_font_cn_${size}.c`);
const conv = path.join(root, 'node_modules', 'lv_font_conv', 'lv_font_conv.js');

if (!fs.existsSync(conv)) {
    console.error(`✗ 找不到 lv_font_conv：${conv}\n  请先在 ${root} 下执行: npm install lv_font_conv`);
    process.exit(1);
}
if (!fs.existsSync(symbolsFile)) {
    console.error(`✗ 找不到符号表：${symbolsFile}`);
    process.exit(1);
}
if (!fs.existsSync(fontTtf)) {
    console.error(`✗ 找不到字体文件：${fontTtf}\n  可用 CN_FONT 环境变量指定其他 ttf`);
    process.exit(1);
}

const symbols = fs.readFileSync(symbolsFile, 'utf8').replace(/\s/g, '');
if (!symbols.length) {
    console.error('✗ 符号表是空的');
    process.exit(1);
}

const args = [
    conv,
    '--font', fontTtf,
    /* ★ 必须显式带上 ASCII 区（0x20~0x7F）★
     * 只给 `--symbols <汉字>` 的话，生成的字体**只有汉字**；
     * 一旦把它设成 `LV_FONT_DEFAULT`，数字和字母就会全变方框。
     * ASCII 区只有 96 个字形，代价很小。 */
    '--range', '0x20-0x7F',
    '--symbols', symbols,
    '--size', String(size),
    '--bpp', String(bpp),
    '--format', 'lvgl',
    '--lv-include', 'lvgl.h',
    '--lv-font-name', `lv_font_cn_${size}`,
    '--no-compress',
    '--force-fast-kern-format',
    '-o', outFile,
];

console.log(`字体 : ${fontTtf}`);
console.log(`字号 : ${size}px, ${bpp}bpp, 稀疏字库`);
console.log(`汉字 : ${symbols.length} 个`);
console.log(`输出 : ${outFile}`);

try {
    execFileSync(process.execPath, args, { stdio: 'inherit' });
} catch (e) {
    console.error('\n✗ lv_font_conv 执行失败');
    process.exit(1);
}

const kb = (fs.statSync(outFile).size / 1024).toFixed(1);
console.log(`\n✓ 生成成功：${kb} KB`);
