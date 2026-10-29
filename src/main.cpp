#include "motion.hpp"
#include "bonus.hpp"
#include <algorithm>
#include <array>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

static Motion loadMotion(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open motion: " + path);
    std::string magic; int version=0, count=0, frameCount=0;
    in >> magic >> version >> count >> frameCount;
    if (magic != "HW2MOTION" || version != 1 || count < 1 || frameCount < 1)
        throw std::runtime_error("Invalid motion header");
    Motion m; in >> m.frameTime;
    for (int i=0; i<count; ++i) {
        Joint j; in >> j.name >> j.parent >> j.offset.x >> j.offset.y >> j.offset.z;
        if (!in || j.parent >= i || j.parent < -1) throw std::runtime_error("Invalid source hierarchy");
        m.skeleton.joints.push_back(j);
    }
    for (int f=0; f<frameCount; ++f) {
        Pose p; p.localRotation.resize(count);
        for (int j=0; j<count; ++j) {
            Vec3 t; Quat& q=p.localRotation[j];
            in >> t.x >> t.y >> t.z >> q.w >> q.x >> q.y >> q.z;
            if (!in) throw std::runtime_error("Truncated motion frame " + std::to_string(f));
            if (j==0) p.rootTranslation=t;
        }
        m.frames.push_back(std::move(p));
    }
    return m;
}

static Skeleton loadRig(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Cannot open rig: " + path);
    std::string magic; int version=0, count=0;
    in >> magic >> version >> count;
    if (magic != "HW2RIG" || version != 1 || count < 1)
        throw std::runtime_error("Invalid rig header");
    Skeleton s;
    for (int i=0; i<count; ++i) {
        Joint j; in >> j.name >> j.parent >> j.offset.x >> j.offset.y >> j.offset.z;
        if (!in || j.parent >= i || j.parent < -1) throw std::runtime_error("Invalid target hierarchy");
        s.joints.push_back(j);
    }
    return s;
}

static int findJoint(const Skeleton& s, const std::string& name) {
    for (size_t i=0; i<s.joints.size(); ++i)
        if (s.joints[i].name==name) return static_cast<int>(i);
    throw std::runtime_error("Missing joint: " + name);
}

static void writeSkeleton(std::ostream& out, const Skeleton& s) {
    out << '[';
    for (size_t i=0; i<s.joints.size(); ++i) {
        if (i) out << ',';
        out << s.joints[i].parent;
    }
    out << ']';
}

static void writePositions(std::ostream& out, const WorldPose& p) {
    out << '[';
    for (size_t i=0; i<p.position.size(); ++i) {
        if (i) out << ',';
        const Vec3& v=p.position[i];
        out << '[' << v.x << ',' << v.y << ',' << v.z << ']';
    }
    out << ']';
}

static WorldPose solveRetargetedPose(const Skeleton& target,
                                    const Pose& transferred,
                                    std::array<int, 2> targetFeet,
                                    const FootPair& goals) {
    Pose pose = transferred;
    solveCCD(target, pose, targetFeet[0], goals[0], 30);
    solveCCD(target, pose, targetFeet[1], goals[1], 30);
    return forwardKinematics(target, pose);
}

static void writeViewer(const std::string& path, const Motion& motion,
                        const Skeleton& target, int count) {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("Cannot write viewer: " + path);
    out << std::fixed << std::setprecision(5);
    out << R"HTML(<!doctype html><html lang="en"><meta charset="utf-8">
<title>HW2 Motion Retargeting</title><style>
body{margin:0;background:#111827;color:#e5e7eb;font:16px system-ui,sans-serif}
header{padding:18px 24px;background:#1f2937}h1{margin:0;font-size:23px}
p{margin:8px 0 0;color:#aebdcb}main{max-width:1250px;margin:auto;padding:20px}
canvas{width:100%;background:#101a2d;border:1px solid #475569;border-radius:8px}
#detail{height:220px;margin-top:12px}
.controls{display:flex;gap:14px;align-items:center;margin-top:16px;flex-wrap:wrap}
button,input{accent-color:#38bdf8}button{background:#38bdf8;border:0;border-radius:5px;padding:8px 14px;font-weight:bold}
#scrub{flex:1;min-width:240px}.legend{display:flex;gap:30px;margin-top:12px}
.options{display:flex;gap:24px;margin-top:16px;flex-wrap:wrap;align-items:center}
.options label{cursor:pointer}.options input{width:17px;height:17px;vertical-align:middle}
#mode{color:#aebdcb;font-size:14px}
.compare{display:flex;gap:10px;margin-top:12px;flex-wrap:wrap}
.compare button{background:#334155;color:#e5e7eb;border:1px solid #64748b}
.hint{font-size:13px;color:#94a3b8;margin-top:9px}
</style><header><h1>HW2 - Motion retargeting</h1><p>Source A and target B have different proportions. Bonus functions are TODOs in src/bonus.cpp.</p></header>
<main><canvas id="view" width="1200" height="620"></canvas><div class="legend"><span style="color:#38bdf8">&#9679; Source A</span><span style="color:#fbbf24">&#9679; Target B</span><span style="color:#e879f9">- - Baseline ghost</span><span style="color:#5eead4">- - Hard-lock ghost</span><span style="color:#86efac">&#9675; Planted-foot anchor</span></div>
<div class="controls"><button id="play">Pause</button><label>Frame <span id="number">0</span></label><input id="scrub" type="range" min="0" max=")HTML";
    out << count-1;
    out << R"HTML(" value="0"><label>View angle <input id="yaw" type="range" min="-180" max="180" value="25"></label></div>
<div class="options"><label><input id="lock" type="checkbox" checked> Stance-aware foot lock</label>
<label><input id="smooth" type="checkbox" checked> Smooth contact transitions</label>
<span id="mode"></span></div>
<div class="compare"><button id="compareLock">Compare foot lock</button><button id="compareSmooth">Compare smoothing</button></div>
<p class="hint">The grid stays in world space. Colored foot trails show drift during the current stance; comparison buttons pause at the clearest frame.</p><canvas id="detail" width="1200" height="220" aria-label="Foot contact comparison in centimeters"></canvas></main><script>
const parentsA=)HTML"; writeSkeleton(out,motion.skeleton);
    out << ";\nconst parentsB="; writeSkeleton(out,target);
    out << ";\nconst A=[\n";
    const int leftA=findJoint(motion.skeleton,"LeftFoot");
    const int rightA=findJoint(motion.skeleton,"RightFoot");
    const int leftB=findJoint(target,"B_LeftFoot");
    const int rightB=findJoint(target,"B_RightFoot");
    std::vector<WorldPose> sourceWorld; sourceWorld.reserve(count);
    for (int f=0; f<count; ++f) {
        sourceWorld.push_back(forwardKinematics(motion.skeleton,motion.frames[f]));
        if (f) out << ",\n";
        writePositions(out,sourceWorld.back());
    }
    std::vector<Pose> transferred;
    std::vector<FootPair> baseGoals;
    transferred.reserve(count);
    baseGoals.reserve(count);
    for (int f=0; f<count; ++f) {
        Pose p=retargetFrame(motion.skeleton,target,motion.frames[f],1.12);
        if (p.localRotation.size()!=target.joints.size())
            throw std::runtime_error("retargetFrame returned incorrect joint count");
        const Vec3 rootA=sourceWorld[f].position[0];
        const Vec3 rootB=p.rootTranslation+target.joints[0].offset;
        baseGoals.push_back({rootB+(sourceWorld[f].position[leftA]-rootA)*1.12,
                             rootB+(sourceWorld[f].position[rightA]-rootA)*1.12});
        transferred.push_back(std::move(p));
    }
    const FootLockResult locked = applyStanceAwareFootLock(
        sourceWorld,baseGoals,{leftA,rightA},motion.frameTime);
    const auto smoothGoals = applySmoothContactTransitions(baseGoals,locked.contacts);
    if (locked.goals.size()!=static_cast<size_t>(count) ||
        locked.contacts.size()!=static_cast<size_t>(count) ||
        smoothGoals.size()!=static_cast<size_t>(count))
        throw std::runtime_error("Bonus functions returned an incorrect frame count");
    const auto& contacts=locked.contacts;
    const std::array<int,2> targetFeet{leftB,rightB};
    out << "\n];\nconst B_base=[\n";
    for (int f=0; f<count; ++f) {
        if (f) out << ",\n";
        writePositions(out,solveRetargetedPose(target,transferred[f],
                                               targetFeet,baseGoals[f]));
    }
    out << "\n];\nconst B_lock=[\n";
    for (int f=0; f<count; ++f) {
        if (f) out << ",\n";
        writePositions(out,solveRetargetedPose(target,transferred[f],
                                               targetFeet,locked.goals[f]));
    }
    out << "\n];\nconst B_smooth=[\n";
    for (int f=0; f<count; ++f) {
        if (f) out << ",\n";
        writePositions(out,solveRetargetedPose(target,transferred[f],
                                               targetFeet,smoothGoals[f]));
    }
    out << "\n];\nconst stance=[\n";
    for (int f=0; f<count; ++f) {
        if (f) out << ',';
        out << '[' << (contacts[f][0].planted ? "true" : "false") << ','
            << (contacts[f][1].planted ? "true" : "false") << ']';
    }
    out << "\n];\nconst anchors=[\n";
    for (int f=0; f<count; ++f) {
        if (f) out << ',';
        out << '[';
        for (size_t side=0; side<2; ++side) {
            if (side) out << ',';
            const Contact& contact=contacts[f][side];
            if (contact.planted) {
                const Vec3 p=contact.anchor;
                out << '[' << p.x << ',' << p.y << ',' << p.z << ']';
            } else {
                out << "null";
            }
        }
        out << ']';
    }
    out << "\n];\nconst feetB=[" << leftB << ',' << rightB << "]";
    out << R"HTML(;
const canvas=document.querySelector('#view'),ctx=canvas.getContext('2d');
const detail=document.querySelector('#detail'),dc=detail.getContext('2d');
const scrub=document.querySelector('#scrub'),number=document.querySelector('#number');
const lock=document.querySelector('#lock'),smooth=document.querySelector('#smooth');
let frame=0,playing=false,last=0;
document.querySelector('#play').textContent='Play';
document.querySelector('#play').onclick=()=>{playing=!playing;document.querySelector('#play').textContent=playing?'Pause':'Play'};
scrub.oninput=()=>{frame=+scrub.value;playing=false;document.querySelector('#play').textContent='Play';draw()};
document.querySelector('#yaw').oninput=draw;
lock.onchange=draw;smooth.onchange=draw;
function distance(a,b){return Math.hypot(a[0]-b[0],a[1]-b[1],a[2]-b[2])}
function frameDifference(first,second,f){return Math.max(...feetB.map(i=>distance(first[f][i],second[f][i])))}
function strongestFrame(first,second){let best=0;
  for(let f=1;f<first.length;f++)if(frameDifference(first,second,f)>frameDifference(first,second,best))best=f;
  return best}
const bestLock=strongestFrame(B_base,B_lock),bestSmooth=strongestFrame(B_lock,B_smooth);
function jumpTo(f,withSmooth){frame=f;playing=false;lock.checked=true;smooth.checked=withSmooth;
  document.querySelector('#play').textContent='Play';draw()}
document.querySelector('#compareLock').onclick=()=>jumpTo(bestLock,false);
document.querySelector('#compareSmooth').onclick=()=>jumpTo(bestSmooth,true);
function project(p,root,center){let x=p[0]-root[0],z=p[2]-root[2];
  let a=(+document.querySelector('#yaw').value)*Math.PI/180;
  return [center+(x*Math.cos(a)-z*Math.sin(a))*500,560-p[1]*500+(x*Math.sin(a)+z*Math.cos(a))*70]}
function grid(root,center){ctx.save();ctx.beginPath();ctx.rect(center-296,70,592,500);ctx.clip();
  for(let axis=0;axis<2;axis++){
    let origin=axis===0?root[0]:root[2];
    for(let k=Math.floor(origin*10)-12;k<=Math.floor(origin*10)+12;k++){
      let v=k/10,p=axis===0?[v,0,root[2]-1]:[root[0]-1,0,v];
      let q=axis===0?[v,0,root[2]+1]:[root[0]+1,0,v];
      let a=project(p,root,center),b=project(q,root,center);
      ctx.strokeStyle=k%5===0?'#3b526c':'#25374f';ctx.lineWidth=k%5===0?1.5:1;
      ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();
    }
  }
  ctx.restore()}
function figure(points,parents,center,color){
  let root=points[0];
  ctx.strokeStyle=color;ctx.lineWidth=3;ctx.lineCap='round';ctx.beginPath();
  for(let j=1;j<points.length;j++){let k=parents[j];if(k<0)continue;
    let x=project(points[j],root,center),p=project(points[k],root,center);ctx.moveTo(p[0],p[1]);ctx.lineTo(x[0],x[1])}
  ctx.stroke();ctx.fillStyle=color;
  for(let p of points){let x=project(p,root,center);ctx.beginPath();ctx.arc(x[0],x[1],3,0,Math.PI*2);ctx.fill()}
}
function ghostLegs(points,root,color){ctx.save();ctx.strokeStyle=color;ctx.lineWidth=4;ctx.setLineDash([7,5]);
  for(let foot of feetB){let j=foot;
    for(let link=0;link<2;link++){let parent=parentsB[j];if(parent<0)break;
      let a=project(points[j],root,900),b=project(points[parent],root,900);
      ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...b);ctx.stroke();j=parent}}
  ctx.restore()}
function drawAnchors(points){ctx.save();ctx.strokeStyle='#86efac';ctx.lineWidth=2;
  for(let side=0;side<2;side++){let anchor=anchors[frame][side];if(!anchor)continue;
    let a=project(anchor,points[0],900),foot=project(points[feetB[side]],points[0],900);
    ctx.setLineDash([3,4]);ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(...foot);ctx.stroke();
    ctx.setLineDash([]);ctx.beginPath();ctx.arc(...a,8,0,Math.PI*2);ctx.stroke()}
  ctx.restore()
}
function footTrail(frames,side,root,color){let start=frame;
  while(start>0&&stance[start-1][side])start--;
  if(frame-start<2)return;
  ctx.save();ctx.strokeStyle=color;ctx.lineWidth=3;ctx.beginPath();
  for(let f=start;f<=frame;f++){let p=project(frames[f][feetB[side]],root,900);
    if(f===start)ctx.moveTo(...p);else ctx.lineTo(...p)}
  ctx.stroke();ctx.restore()}
function contactWindow(){let nearest=-1,side=0,delta=Infinity;
  for(let f=0;f<stance.length;f++)for(let s=0;s<2;s++)if(stance[f][s]){
    let d=Math.abs(f-frame);if(d<delta){nearest=f;side=s;delta=d}}
  if(nearest<0)return null;
  let start=nearest,end=nearest;
  while(start>0&&stance[start-1][side])start--;
  while(end+1<stance.length&&stance[end+1][side])end++;
  return {start,end,side}}
function horizontalDrift(frames,f,side,anchor){let foot=frames[f][feetB[side]];
  return Math.hypot(foot[0]-anchor[0],foot[2]-anchor[2])*100}
function drawDetail(selected){dc.clearRect(0,0,1200,220);dc.fillStyle='#e5e7eb';dc.font='16px system-ui';
  let win=contactWindow();if(!win){dc.fillText('No stance interval in this clip',28,34);return}
  const {start,end,side}=win,anchor=anchors[start][side],series=[
    {data:B_base,color:'#e879f9',name:'Lock off'},
    {data:B_lock,color:'#5eead4',name:'Hard lock'},
    {data:B_smooth,color:'#fbbf24',name:'Smooth lock'}];
  let max=5;for(let mode of series)for(let f=start;f<=end;f++)
    max=Math.max(max,horizontalDrift(mode.data,f,side,anchor));
  max=Math.ceil(max/2)*2;
  const left=58,right=900,top=46,bottom=181;
  dc.fillText((side===0?'Left':'Right')+' foot: distance from planted anchor',28,27);
  dc.fillStyle='#94a3b8';dc.font='13px system-ui';dc.fillText('cm',28,52);
  for(let n=0;n<=4;n++){let y=bottom-(bottom-top)*n/4;
    dc.strokeStyle='#334155';dc.lineWidth=1;dc.beginPath();dc.moveTo(left,y);dc.lineTo(right,y);dc.stroke();
    dc.fillStyle='#94a3b8';dc.fillText((max*n/4).toFixed(1),19,y+4)}
  const x=f=>left+(f-start)/(Math.max(1,end-start))*(right-left);
  if(frame>=start&&frame<=end){dc.strokeStyle='#64748b';dc.setLineDash([4,4]);
    dc.beginPath();dc.moveTo(x(frame),top);dc.lineTo(x(frame),bottom);dc.stroke();dc.setLineDash([])}
  for(let mode of series){dc.strokeStyle=mode.color;dc.lineWidth=mode.data===selected?4:2;
    dc.beginPath();for(let f=start;f<=end;f++){
      let y=bottom-horizontalDrift(mode.data,f,side,anchor)/max*(bottom-top);
      if(f===start)dc.moveTo(x(f),y);else dc.lineTo(x(f),y)}dc.stroke();
    if(frame>=start&&frame<=end){let y=bottom-horizontalDrift(mode.data,frame,side,anchor)/max*(bottom-top);
      dc.fillStyle=mode.color;dc.beginPath();dc.arc(x(frame),y,5,0,Math.PI*2);dc.fill()}}
  dc.fillStyle='#94a3b8';dc.fillText('stance frames '+start+'–'+end,58,207);
  series.forEach((mode,i)=>{let y=69+i*48,now=Math.min(end,Math.max(start,frame));
    dc.fillStyle=mode.color;dc.fillRect(936,y-12,13,13);dc.font='15px system-ui';
    dc.fillText(mode.name,960,y);dc.font='bold 20px system-ui';
    dc.fillText(horizontalDrift(mode.data,now,side,anchor).toFixed(1)+' cm',960,y+23)})}
function draw(){ctx.clearRect(0,0,1200,620);ctx.fillStyle='#94a3b8';ctx.font='18px system-ui';
  smooth.disabled=!lock.checked;
  const targetFrames=!lock.checked?B_base:(smooth.checked?B_smooth:B_lock);
  const label=!lock.checked?'Foot lock off':(smooth.checked?'Foot lock + smooth transitions':'Foot lock on, hard transitions');
  const planted=stance[frame];
  const shift=frameDifference(B_base,targetFrames,frame)*100;
  const transition=frameDifference(B_lock,B_smooth,frame)*100;
  document.querySelector('#mode').textContent=label+' | Stance: '+(planted[0]?'left ':'')+(planted[1]?'right':'')+(!planted[0]&&!planted[1]?'none':'')+' | Lock shift '+shift.toFixed(1)+' cm | Smoothing shift '+transition.toFixed(1)+' cm';
  ctx.fillText('Source A',260,42);ctx.fillText('Target B',860,42);
  grid(A[frame][0],300);grid(targetFrames[frame][0],900);
  ctx.strokeStyle='#475569';ctx.beginPath();ctx.moveTo(600,0);ctx.lineTo(600,620);ctx.stroke();
  if(lock.checked)ghostLegs(B_base[frame],targetFrames[frame][0],'#e879f9');
  if(lock.checked&&smooth.checked)ghostLegs(B_lock[frame],targetFrames[frame][0],'#5eead4');
  figure(A[frame],parentsA,300,'#38bdf8');figure(targetFrames[frame],parentsB,900,'#fbbf24');
  for(let side=0;side<2;side++)if(planted[side]){
    footTrail(B_base,side,targetFrames[frame][0],'#e879f9');
    footTrail(B_lock,side,targetFrames[frame][0],'#5eead4');
    footTrail(targetFrames,side,targetFrames[frame][0],'#fbbf24')}
  drawAnchors(targetFrames[frame]);
  drawDetail(targetFrames);scrub.value=frame;number.textContent=frame}
function tick(t){if(playing&&t-last>)HTML";
    out << motion.frameTime*1000;
    out << R"HTML(){frame=(frame+1)%A.length;last=t;draw()}requestAnimationFrame(tick)}
frame=bestSmooth;draw();requestAnimationFrame(tick);
</script></html>)HTML";
}

int main(int argc, char** argv) {
    try {
        if (argc!=4 && argc!=6) {
            std::cerr << "Usage: hw2 <source.motion> <target.rig> <viewer.html> [--frames N]\n";
            return 2;
        }
        Motion motion=loadMotion(argv[1]);
        Skeleton target=loadRig(argv[2]);
        int count=static_cast<int>(motion.frames.size());
        if (argc==6) {
            if (std::string(argv[4])!="--frames") throw std::runtime_error("Expected --frames");
            count=std::stoi(argv[5]);
            if (count<1 || count>static_cast<int>(motion.frames.size()))
                throw std::runtime_error("Frame count out of range");
        }
        writeViewer(argv[3],motion,target,count);
        std::cout << "Wrote " << argv[3] << " (" << count << " frames, "
                  << motion.skeleton.joints.size() << " joints, "
                  << motion.frameTime << " s/frame)\n";
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
}
