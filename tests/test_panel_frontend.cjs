// FEATURE: wheel zoom, collapsible/expandable panels and independent model heading.
// SPEC: retain lidar projection; rotate only the model; layout never sends commands.
// IMPLEMENTATION: independent heading offset, bounded wheel zoom and panel state controls.
// RESULT: IMPLEMENTED — unit regressions and viewport/panel browser checks pass.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(process.argv[2] || path.join(__dirname, '../ros2/panel.html'), 'utf8');
const code = html.match(/<script>([\s\S]*?)<\/script>/)[1];

function browser() {
  const listeners = {}, elements = {}, posts = [], intervals = [], timeouts = [];
  const listenerOptions = {}, drawOps = [];
  const ctx = new Proxy({}, {
    get: (_, key) => key === 'canvas' ? {} : (...args) => drawOps.push([key, ...args]),
  });
  function element(id) {
    if (elements[id]) return elements[id];
    const classes = new Set(), attrs = {};
    return elements[id] = {
      textContent: '', style: {}, value: '3', hidden: false, dataset: {},
      classList: {
        contains: value => classes.has(value),
        toggle(value, enabled) {
          if (enabled === undefined) enabled = !classes.has(value);
          if (enabled) classes.add(value); else classes.delete(value);
          return enabled;
        },
      },
      setAttribute: (name, value) => attrs[name] = String(value),
      getAttribute: name => attrs[name],
      focus() { this.focused = true; },
      addEventListener(name, fn, options) {
        listeners[id + ':' + name] = fn;
        listenerOptions[id + ':' + name] = options;
      },
      getContext: () => ctx,
      getBoundingClientRect: () => ({width: 600, height: 400}),
      insertAdjacentHTML() {}, remove() {},
    };
  }
  const sandbox = {
    console, Math, Number, Date, JSON, Set, devicePixelRatio: 1,
    Image: class { set src(value) { this.value = value; this.onload?.(); } },
    document: {
      getElementById: element,
      addEventListener: (name, fn) => listeners['document:' + name] = fn,
    },
    addEventListener: (name, fn) => {
      const previous = listeners[name];
      listeners[name] = event => { previous?.(event); fn(event); };
    },
    ResizeObserver: class { observe() {} },
    setInterval: fn => intervals.push(fn),
    setTimeout: fn => timeouts.push(fn), clearTimeout() {},
    fetch(url, options) {
      if (options?.method === 'POST') posts.push(JSON.parse(options.body));
      return new Promise(() => {});
    },
  };
  vm.createContext(sandbox);
  vm.runInContext(code, sandbox);
  const key = (name, code) => listeners[name]({
    code, target: {tagName: 'BODY'}, preventDefault() {},
  });
  return {listeners, listenerOptions, elements, posts, intervals, timeouts, key, sandbox, drawOps};
}

function showScan(b) {
  vm.runInContext('lidar={points:[[1,0],[0,1]]}', b.sandbox);
  b.sandbox.renderState({sensors: {}, odom: {x: 0, y: 0, yaw: 0}});
}
function dots(b) {
  return b.drawOps.filter(op => op[0] === 'arc' && op[3] === 2);
}
function click(b, id) {
  assert.equal(typeof b.listeners[id + ':click'], 'function', id + ' needs a click handler');
  b.listeners[id + ':click']();
}
function wheel(b, deltaY, deltaMode = 0) {
  assert.equal(typeof b.listeners['scan:wheel'], 'function', 'map needs wheel zoom');
  let prevented = false;
  b.listeners['scan:wheel']({deltaY, deltaMode, preventDefault() { prevented = true; }});
  assert.ok(prevented, 'map wheel must not scroll the page');
}
// FEATURE: occupancy map is primary; pose and optional scan must share its frame.
// IMPLEMENTATION: versioned PNG, map-frame overlays, fit/pan and scan toggle.
// RESULT: IMPLEMENTED — unit regressions and offline browser interactions pass.
const tests = {
  map_uses_origin_yaw_and_resolution_without_raw_odom_overlay() {
    const b = browser();
    assert.equal(typeof b.sandbox.renderMap, 'function');
    b.sandbox.renderMap({width:2,height:3,resolution:.05,origin:{x:4,y:5,yaw:.3},frame:'map',version:1,image:'data:image/png;base64,test'});
    b.drawOps.length=0;
    b.sandbox.renderState({sensors:{},odom:{x:100,y:100,yaw:0},map_age_s:0,map_pose:{x:4,y:5,yaw:.5,frame:'map'}});
    assert.ok(b.drawOps.some(op=>op[0]==='drawImage'), 'occupancy image must be drawn');
    assert.ok(b.drawOps.some(op=>op[0]==='rotate'&&op[1]===-.3), 'grid origin rotation');
    assert.equal(b.drawOps.filter(op=>op[0]==='rotate').at(-1)[1], -.5-Math.PI/2);
    assert.equal(b.elements['pose-x'].textContent,'4.00 м');
    b.drawOps.length=0;
    b.sandbox.renderState({sensors:{},odom:{x:100,y:100,yaw:0},map_age_s:0});
    assert.equal(b.drawOps.filter(op=>op[0]==='arc').length,0,'no robot without map TF');
    assert.match(b.elements['map-message'].textContent,/TF/);
    assert.equal(b.posts.length,0);
  },
  map_frame_mismatch_hides_scan_and_pose_and_restart_clears_map() {
    const b=browser();
    b.sandbox.renderMap({width:2,height:2,resolution:.05,origin:{x:0,y:0,yaw:0},frame:'map',version:1,image:'data:image/png;base64,test'});
    vm.runInContext("lidar={frame:'odom',points:[[1,0]]}",b.sandbox);
    b.drawOps.length=0;
    b.sandbox.renderState({sensors:{},odom:{x:100,y:100,yaw:0},map_pose:{x:100,y:100,yaw:0,frame:'odom'}});
    assert.equal(dots(b).length,0,'odom scan must not appear on map');
    assert.equal(b.elements['pose-x'].textContent,'—','mismatched pose must stay missing');
    b.sandbox.renderMap({});
    assert.equal(vm.runInContext('occupancy',b.sandbox),null,'empty map after restart clears old map');
  },
  map_fit_pan_and_scan_toggle_are_passive() {
    const b=browser();
    b.sandbox.renderMap({width:100,height:100,resolution:.05,origin:{x:-2,y:-2,yaw:0},frame:'map',version:1,image:'data:image/png;base64,test'});
    click(b,'map-fit');
    const center=vm.runInContext('({...viewCenter})',b.sandbox);
    assert.equal(center.x,.5); assert.equal(center.y,.5);
    b.listeners['scan:pointerdown']({button:0,pointerId:1,clientX:10,clientY:10});
    b.listeners['scan:pointermove']({pointerId:1,clientX:60,clientY:10});
    b.listeners['scan:pointerup']({pointerId:1});
    assert.ok(vm.runInContext('viewCenter.x',b.sandbox)<center.x);
    click(b,'map-scan');
    assert.equal(b.elements['map-scan'].getAttribute('aria-pressed'),'false');
    assert.equal(b.posts.length,0);
  },
  map_age_and_missing_map_are_explicit() {
    const b=browser();
    assert.match(b.elements['map-message'].textContent,/Ожидание карты/);
    b.sandbox.renderMap({width:2,height:2,resolution:1,origin:{x:0,y:0,yaw:0},frame:'map',version:1,image:'data:image/png;base64,test'});
    b.sandbox.renderState({sensors:{},map_age_s:12,map_pose:{x:0,y:0,yaw:0,frame:'map'}});
    assert.match(b.elements['map-message'].textContent,/Нет обновлений/);
  },
  lidar_keeps_original_orientation_while_model_turns() {
    const b = browser(); showScan(b);
    const points = dots(b);
    assert.ok(points[0][1] > 300, 'positive lidar x stays to the right');
    assert.equal(points[0][2], 200, 'lidar must not rotate together with model');
    assert.equal(points[1][1], 300);
    assert.ok(points[1][2] < 200, 'positive lidar y stays up');
    assert.equal(b.drawOps.filter(op => op[0] === 'rotate').at(-1)[1], -Math.PI / 2);
  },
  model_heading_correction_preserves_yaw_changes() {
    const b = browser();
    b.sandbox.renderState({sensors: {}, odom: {x: 0, y: 0, yaw: 0}});
    assert.equal(b.drawOps.filter(op => op[0] === 'rotate').at(-1)[1], -Math.PI / 2);
    b.sandbox.renderState({sensors: {}, odom: {x: 0, y: 0, yaw: Math.PI / 2}});
    assert.equal(b.drawOps.filter(op => op[0] === 'rotate').at(-1)[1], -Math.PI);
  },
  wheel_changes_drawn_scale_and_never_publishes_motion() {
    const b = browser(); showScan(b);
    const before = dots(b)[0][1] - 300;
    b.drawOps.length = 0; wheel(b, -120);
    const after = dots(b)[0][1] - 300;
    assert.ok(after > before, 'wheel up must zoom in');
    b.drawOps.length = 0; wheel(b, 120);
    assert.ok(Math.abs(dots(b)[0][1] - 300 - before) < 1e-8, 'opposite wheel restores scale');
    assert.equal(b.listenerOptions['scan:wheel'].passive, false);
    assert.equal(b.posts.length, 0);
    assert.ok(!html.includes('<select'), 'radius selector must be removed');
  },
  wheel_has_finite_limits_and_normalizes_event_units() {
    const b = browser(); showScan(b);
    wheel(b, -1e9);
    assert.equal(vm.runInContext('viewRadius', b.sandbox), .35);
    wheel(b, 1e9);
    assert.equal(vm.runInContext('viewRadius', b.sandbox), 30);
    const bounded = vm.runInContext('viewRadius', b.sandbox);
    wheel(b, NaN);
    assert.equal(vm.runInContext('viewRadius', b.sandbox), bounded);
    const pixels = browser(), lines = browser(), pages = browser();
    wheel(pixels, 16); wheel(lines, 1, 1);
    assert.equal(vm.runInContext('viewRadius', pixels.sandbox), vm.runInContext('viewRadius', lines.sandbox));
    const pagePixels = browser(); wheel(pagePixels, 400); wheel(pages, 1, 2);
    assert.equal(vm.runInContext('viewRadius', pagePixels.sandbox), vm.runInContext('viewRadius', pages.sandbox));
  },
  each_panel_collapses_and_reopens_without_motion() {
    const b = browser();
    for (const id of ['map', 'camera', 'sensors']) {
      click(b, id + '-collapse');
      assert.ok(b.elements['panel-' + id].classList.contains('is-collapsed'));
      assert.equal(b.elements[id + '-collapse'].getAttribute('aria-expanded'), 'false');
      assert.equal(b.elements.workspace.dataset[id + 'Collapsed'], 'true');
      click(b, id + '-collapse');
      assert.ok(!b.elements['panel-' + id].classList.contains('is-collapsed'));
      assert.equal(b.elements[id + '-collapse'].getAttribute('aria-expanded'), 'true');
    }
    assert.equal(b.posts.length, 0);
  },
  expand_is_exclusive_and_escape_restores_layout() {
    const b = browser();
    click(b, 'camera-collapse'); click(b, 'camera-expand');
    assert.ok(!b.elements['panel-camera'].classList.contains('is-collapsed'));
    assert.ok(b.elements['panel-camera'].classList.contains('is-expanded'));
    assert.equal(b.elements['camera-expand'].getAttribute('aria-pressed'), 'true');
    click(b, 'map-expand');
    assert.ok(!b.elements['panel-camera'].classList.contains('is-expanded'));
    assert.equal(b.elements.workspace.dataset.expanded, 'map');
    b.key('keydown', 'Escape');
    assert.equal(b.elements.workspace.dataset.expanded, '');
    assert.ok(!b.elements['panel-map'].classList.contains('is-expanded'));
    assert.equal(b.elements['map-expand'].focused, true);
    click(b, 'sensors-expand'); click(b, 'sensors-expand');
    assert.equal(b.elements.workspace.dataset.expanded, '');
    assert.equal(b.posts.length, 0);
  },
  collapsing_expanded_panel_restores_workspace() {
    const b = browser(); click(b, 'map-expand'); click(b, 'map-collapse');
    assert.equal(b.elements.workspace.dataset.expanded, '');
    assert.ok(b.elements['panel-map'].classList.contains('is-collapsed'));
  },
  passive_view_does_not_publish() {
    const b = browser(); b.listeners.blur(); assert.equal(b.posts.length, 0);
  },
  keyboard_uses_physical_limits() {
    const b = browser(); b.key('keydown', 'KeyW'); assert.equal(b.posts.at(-1).v, .1);
    b.key('keydown', 'KeyA'); assert.equal(b.posts.at(-1).w, .4);
    b.key('keyup', 'KeyW'); assert.equal(b.posts.at(-1).v, 0);
    b.key('keyup', 'KeyA'); assert.equal(b.posts.at(-1).w, 0);
  },
  held_key_refreshes_and_blur_releases() {
    const b = browser(); b.key('keydown', 'KeyW'); const start = b.posts.length;
    b.intervals.forEach(fn => fn()); assert.ok(b.posts.length > start);
    b.listeners.blur(); assert.equal(b.posts.at(-1).v, 0);
    const stopped = b.posts.length; b.intervals.forEach(fn => fn());
    assert.equal(b.posts.length, stopped);
  },
  missing_sensors_stay_visible() {
    const b = browser(); b.sandbox.renderState({sensors: {}});
    for (const id of ['battery', 'bumper', 'wall', 'cliff-0', 'cliff-1', 'cliff-2', 'cliff-3', 'drop-0', 'drop-1', 'sonar']) {
      assert.equal(b.elements[id].textContent, 'нет данных', id);
    }
  },
  voltage_and_unknown_range_are_not_conflated() {
    const b = browser(); b.sandbox.renderState({sensors: {cliff_ir: [0, 0, 0, 0]}});
    assert.equal(b.elements['cliff-0'].textContent, '— м / 0.00 V');
  },
  camera_failure_retries() {
    const b = browser(); b.elements.cam.onerror();
    assert.equal(b.elements['cam-status'].textContent, 'Нет потока');
    assert.ok(b.timeouts.length > 0); b.elements.cam.onload();
    assert.equal(b.elements['cam-off'].hidden, true);
  },
};
let failed = 0;
for (const [name, test] of Object.entries(tests)) {
  try { test(); console.log('PASS ' + name); }
  catch (e) { failed++; console.error('FAIL ' + name + ': ' + e.message); }
}
process.exitCode = failed ? 1 : 0;
