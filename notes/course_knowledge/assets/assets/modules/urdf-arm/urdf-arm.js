/*!
 * urdf-arm.js —— 极简 URDF 加载器（经典 script，只依赖全局 THREE）
 *
 * 为什么自己写：课程页面要能用 file:// 双击打开，ES module 和 fetch 都被
 * 浏览器拦（origin null），所以不能直接用 urdf-loader 那一套 ESM。
 * 这里只实现课件需要的子集：
 *   · <joint> 的 origin(xyz/rpy)、axis、type = fixed | revolute | continuous
 *   · <link><visual> 的 origin + <box> / <cylinder> / <sphere> / <mesh>
 *   · <material><color rgba>（没有就用 colorOf 回调兜底）
 *
 * 用法：
 *   var arm = URDFArm.fromText(urdfText);          // 从字符串
 *   var arm = URDFArm.fromBundle(window.XXX);      // 从 {urdf, meshes} 数据包
 *   scene.add(arm.root);
 *   arm.setJoint('arm_joint1', 0.5);               // 弧度
 *   arm.getLinkWorldPosition('tcp_link', v);
 *
 * 坐标约定：完全按 URDF（ROS 系，Z 向上），不做任何"美化"旋转；
 * 需要 Y 向上时由使用方把 root 转 -90°（本课件就是这么做的）。
 */
(function (global) {
  'use strict';

  function num(s, d) {
    var v = parseFloat(s);
    return isFinite(v) ? v : (d || 0);
  }
  function vec3(s, d) {
    if (!s) return new THREE.Vector3(d || 0, d || 0, d || 0);
    var a = s.trim().split(/\s+/);
    return new THREE.Vector3(num(a[0]), num(a[1]), num(a[2]));
  }
  /* URDF 的 rpy 是固定轴 roll-pitch-yaw，即 R = Rz(yaw)·Ry(pitch)·Rx(roll) */
  function quatFromRPY(rpy) {
    return new THREE.Quaternion().setFromEuler(
      new THREE.Euler(rpy.x, rpy.y, rpy.z, 'ZYX'));
  }

  /* -------- rgba 文本 → 0xRRGGBB（URDF 里是 0~1 浮点） -------- */
  function hexFromRGBA(s) {
    if (!s) return null;
    var a = s.trim().split(/\s+/).map(parseFloat);
    if (a.length < 3 || a.some(function (v) { return !isFinite(v); })) return null;
    var c = function (v) {
      var n = Math.round(Math.max(0, Math.min(1, v)) * 255);
      return ('0' + n.toString(16)).slice(-2);
    };
    return parseInt(c(a[0]) + c(a[1]) + c(a[2]), 16);
  }

  /* -------- 基本体：URDF 的 cylinder 沿 Z，three 的沿 Y -------- */
  function geometryFor(el) {
    var box = el.querySelector('box');
    if (box) {
      var s = vec3(box.getAttribute('size'), 0.05);
      return new THREE.BoxGeometry(Math.max(s.x, 1e-4), Math.max(s.y, 1e-4), Math.max(s.z, 1e-4));
    }
    var cyl = el.querySelector('cylinder');
    if (cyl) {
      var r = Math.max(num(cyl.getAttribute('radius'), 0.02), 1e-4);
      var h = Math.max(num(cyl.getAttribute('length'), 0.05), 1e-4);
      var g = new THREE.CylinderGeometry(r, r, h, 28);
      g.rotateX(Math.PI / 2);        /* Y → Z，与 URDF 的圆柱轴一致 */
      return g;
    }
    var sph = el.querySelector('sphere');
    if (sph) {
      return new THREE.SphereGeometry(Math.max(num(sph.getAttribute('radius'), 0.02), 1e-4), 24, 16);
    }
    return null;                      /* <mesh>：需要外部网格数据，由调用方处理 */
  }

  /* ---------------------------------------------------------------- 主体 */
  function build(urdfText, opts) {
    opts = opts || {};
    var doc = new DOMParser().parseFromString(urdfText, 'text/xml');
    var bad = doc.querySelector('parsererror');
    if (bad) throw new Error('URDF 不是合法 XML：' + bad.textContent.slice(0, 120));
    var robotEl = doc.querySelector('robot');
    if (!robotEl) throw new Error('URDF 里找不到 <robot>');

    var root = new THREE.Group();
    root.name = robotEl.getAttribute('name') || 'robot';

    var colorOf = opts.colorOf || function () { return 0xb8c0cc; };

    var links = {};
    var joints = {};

    /* 1. 建 link 容器（URDF 里每个 link 就是一个坐标系） */
    Array.prototype.forEach.call(robotEl.querySelectorAll('link'), function (el) {
      var g = new THREE.Group();
      g.name = el.getAttribute('name');
      links[g.name] = g;
    });

    /* 2. 填 visual 几何 */
    Array.prototype.forEach.call(robotEl.querySelectorAll('link'), function (el) {
      var linkName = el.getAttribute('name');
      var linkObj = links[linkName];
      if (!linkObj) return;

      Array.prototype.forEach.call(el.querySelectorAll('visual'), function (vEl) {
        var oEl = vEl.querySelector('origin');
        var pos = vec3(oEl && oEl.getAttribute('xyz'));
        var rpy = vec3(oEl && oEl.getAttribute('rpy'));

        var matEl = vEl.querySelector('material color');
        var hex = hexFromRGBA(matEl && matEl.getAttribute('rgba'));
        if (hex === null) hex = colorOf(linkName);

        /* Phong 而不是 Standard：Standard 的 metalness 在没有环境贴图时会让
           背光面发黑，整台臂会读成"几块飘着的板"。Phong 稳定。 */
        var mat = new THREE.MeshPhongMaterial({
          color: hex,
          shininess: 40,
          specular: 0x2a2f36,
          side: THREE.DoubleSide      /* 自建几何也可能有背面朝向，双面最稳 */
        });

        var geo = geometryFor(vEl);
        if (!geo) {
          /* 本加载器只渲染基本体；遇到 <mesh> 会跳过并提示，
             需要网格时请用完整的 urdf-loader + STLLoader 那一套。 */
          var mEl = vEl.querySelector('mesh');
          if (mEl) console.warn('[urdf-arm] 跳过 <mesh>（本加载器只支持基本体）：', mEl.getAttribute('filename'));
          return;
        }

        var mesh = new THREE.Mesh(geo, mat);
        mesh.position.copy(pos);
        mesh.quaternion.copy(quatFromRPY(rpy));
        mesh.name = linkName + '_visual';
        linkObj.add(mesh);
      });
    });

    /* 3. joint：父 link → joint 组（origin + 转角）→ 子 link */
    Array.prototype.forEach.call(robotEl.querySelectorAll('joint'), function (el) {
      var name = el.getAttribute('name');
      var type = el.getAttribute('type');
      var pEl = el.querySelector('parent');
      var cEl = el.querySelector('child');
      if (!pEl || !cEl) return;
      var parentName = pEl.getAttribute('link');
      var childName = cEl.getAttribute('link');
      if (!links[parentName] || !links[childName]) return;

      var oEl = el.querySelector('origin');
      var pos = vec3(oEl && oEl.getAttribute('xyz'));
      var rpy = vec3(oEl && oEl.getAttribute('rpy'));

      var jg = new THREE.Group();
      jg.name = name;
      jg.position.copy(pos);
      var originQuat = quatFromRPY(rpy);
      jg.quaternion.copy(originQuat);

      links[parentName].add(jg);
      jg.add(links[childName]);

      var aEl = el.querySelector('axis');
      var axis = aEl ? vec3(aEl.getAttribute('xyz')) : new THREE.Vector3(1, 0, 0);
      if (axis.lengthSq() === 0) axis.set(0, 0, 1);
      axis.normalize();

      var lEl = el.querySelector('limit');
      joints[name] = {
        name: name, obj: jg, type: type, axis: axis, originQuat: originQuat,
        lower: lEl ? num(lEl.getAttribute('lower'), -Math.PI) : -Math.PI,
        upper: lEl ? num(lEl.getAttribute('upper'), Math.PI) : Math.PI,
        value: 0, parent: parentName, child: childName
      };
    });

    /* 4. 挂到根：找没有被任何 joint 当 child 的 link（通常是 base_footprint） */
    var isChild = {};
    Object.keys(joints).forEach(function (k) { isChild[joints[k].child] = true; });
    Object.keys(links).forEach(function (k) {
      if (!isChild[k]) root.add(links[k]);
    });

    var _q = new THREE.Quaternion();

    return {
      root: root,
      links: links,
      joints: joints,

      /* 可动关节，按 URDF 中出现的顺序 */
      movable: Object.keys(joints)
        .map(function (k) { return joints[k]; })
        .filter(function (j) { return j.type === 'revolute' || j.type === 'continuous'; }),

      setJoint: function (name, value) {
        var j = joints[name];
        if (!j) return;
        j.value = value;
        _q.setFromAxisAngle(j.axis, value);
        j.obj.quaternion.copy(j.originQuat).multiply(_q);
      },

      getLinkWorldPosition: function (name, target) {
        var o = links[name];
        if (!o) return null;
        o.updateWorldMatrix(true, false);
        return (target || new THREE.Vector3()).setFromMatrixPosition(o.matrixWorld);
      },

      /* 实体包围盒（URDF 系，单位 m）。只统计 Mesh，跳过坐标帧箭头和 Sprite
         —— Sprite 的 scale 随相机变，混进来会让取景距离每次都不一样。 */
      getBoundingBox: function () {
        root.updateMatrixWorld(true);
        var box = new THREE.Box3();
        var tmp = new THREE.Box3();
        root.traverse(function (o) {
          if (!o.isMesh) return;
          if (!o.geometry.boundingBox) o.geometry.computeBoundingBox();
          tmp.copy(o.geometry.boundingBox).applyMatrix4(o.matrixWorld);
          box.union(tmp);
        });
        box.applyMatrix4(new THREE.Matrix4().copy(root.matrixWorld).invert());
        return box;
      }
    };
  }

  global.URDFArm = {
    build: build,
    fromText: function (text, opts) { return build(text, opts); }
  };
})(window);
