import fs from 'node:fs';
import vm from 'node:vm';

const root = new URL('../', import.meta.url);
const html = fs.readFileSync(new URL('platform_b/090105.html', root), 'utf8');
const script = fs.readFileSync(new URL('platform_b/platform_b.js', root), 'utf8');
const visibleHtml = html.replace(
  /<script type="text\/plain" id="legacyLogic">[\s\S]*?<\/script>/,
  ''
);

for (const originalText of [
  '骨伤牵引机器人工作站',
  '机械臂状态',
  '牵引力曲线',
  '牵引力调节',
  '目标牵引力',
  '实时牵引力',
  '开始牵引并记录',
  '结束牵引',
  '紧急停止',
]) {
  if (!visibleHtml.includes(originalText)) throw new Error(`缺少最初界面内容：${originalText}`);
}

for (const addedInterface of [
  'guideZero',
  'tareBtn',
  'device-summary',
  'targetPresets',
  'forceControlDetail',
]) {
  if (visibleHtml.includes(addedInterface)) throw new Error(`仍有后来添加的界面：${addedInterface}`);
}

if (!visibleHtml.includes('id="robotViewer"')) throw new Error('原机械臂区域没有接入实时模型');
if (visibleHtml.includes('robotViewerStatus') || visibleHtml.includes('jointAngles')) {
  throw new Error('实时模型区域增加了额外可见状态项');
}
if (!visibleHtml.includes('/assets/platform_b.js')) throw new Error('后台控制脚本没有接入');
new vm.Script(script, { filename: 'platform_b.js' });
console.log('PASS：平台 B 保持最初界面，后台恒力控制脚本语法正常');
