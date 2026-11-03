// pointcloud_viewer.js - Three.js point cloud rendering

(function () {
  'use strict';

  let scene, camera, renderer, controls, points, geometry, material;
  let container;
  let initialized = false;
  const DEFAULT_CAM = { x: 0, y: 0, z: -1.5 };

  function init() {
    if (initialized) return;
    container = document.getElementById('canvas-3d');

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x0d0f12);

    const w = container.clientWidth || 800;
    const h = container.clientHeight || 600;

    camera = new THREE.PerspectiveCamera(60, w / h, 0.05, 100);
    camera.position.set(DEFAULT_CAM.x, DEFAULT_CAM.y, DEFAULT_CAM.z);
    camera.up.set(0, -1, 0);  // camera optical frame: Y down
    camera.lookAt(0, 0, 1);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.setSize(w, h);
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.1;
    controls.target.set(0, 0, 1);

    geometry = new THREE.BufferGeometry();
    material = new THREE.PointsMaterial({
      size: 2,
      sizeAttenuation: false,
      vertexColors: true
    });
    points = new THREE.Points(geometry, material);
    scene.add(points);

    // grid + axes for orientation
    const axes = new THREE.AxesHelper(0.3);
    scene.add(axes);

    window.addEventListener('resize', onResize);
    document.getElementById('reset-view').addEventListener('click', resetView);
    document.getElementById('point-size').addEventListener('input', (e) => {
      material.size = parseFloat(e.target.value);
    });

    initialized = true;
    animate();
  }

  function onResize() {
    if (!initialized) return;
    const w = container.clientWidth, h = container.clientHeight;
    if (w === 0 || h === 0) return;
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    renderer.setSize(w, h);
  }

  function resetView() {
    camera.position.set(DEFAULT_CAM.x, DEFAULT_CAM.y, DEFAULT_CAM.z);
    controls.target.set(0, 0, 1);
    controls.update();
  }

  function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    if (renderer) renderer.render(scene, camera);
  }

  // Decode gzip+base64 payload: uint32 count, float32 xyz*count, uint8 rgb*count
  function update(b64) {
    if (!initialized) return;
    try {
      const compressed = Uint8Array.from(atob(b64), c => c.charCodeAt(0));
      const raw = window.pako.inflate(compressed);
      const dv = new DataView(raw.buffer);

      const count = dv.getUint32(0, true);
      const xyzOffset = 4;
      const rgbOffset = 4 + count * 12;

      const positions = new Float32Array(raw.buffer, xyzOffset, count * 3);
      const rgbBytes = new Uint8Array(raw.buffer, rgbOffset, count * 3);

      const colors = new Float32Array(count * 3);
      for (let i = 0; i < count * 3; i++) {
        colors[i] = rgbBytes[i] / 255;
      }

      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
      geometry.attributes.position.needsUpdate = true;
      geometry.attributes.color.needsUpdate = true;
      geometry.computeBoundingSphere();
    } catch (e) {
      console.error('pointcloud decode error', e);
    }
  }

  function onShow() {
    init();
    setTimeout(onResize, 50);  // container now has size
  }

  window.PointCloudViewer = { update, onShow };
})();
