// Imports
import * as THREE from 'three';
import { GLTFLoader } from './three/examples/jsm/loaders/GLTFLoader.js';
import { OrbitControls } from './three/examples/jsm/controls/OrbitControls.js';

///////////////////////////
//
//    Helper Functions 
//
///////////////////////////
const timer = ms => new Promise(res => setTimeout(res, ms));

function degrees_to_radians(degrees) {
    return degrees * (Math.PI / 180);
}

function setActive(i) {
    if (prev != -1) {
        prev.classList.remove("active");
    }
    prev = document.getElementById(data[i][0]);
    document.getElementById(data[i][0]).classList.add("active");
}

///////////////////
//
//    IMU Data 
//
///////////////////
var data = [['Time_s', 'x', 'y', 'z'],
[7.7, 0.0, 1.3, 0.1],
[11.7, -0.4, 3.8, 0.4],
[13.7, -0.4, 4.5, 0.7],
[19.7, -2.1, 7.8, 1.5],
[21.7, -2.5, 8.5, 1.6],
[25.8, -4.4, 9.8, 2.0],
[31.8, -7.5, 11.4, 2.9],
[37.8, -10.5, 11.8, 3.6],
[41.9, -13.0, 11.6, 3.8],
[51.9, -18.6, 8.1, 4.3],
[62.0, -22.6, 2.0, 4.4]
]

// builds data table
var tab = document.createElement("table");
tab.id = "data";
for (var i = 0; i < data.length; i++) {
    var row = tab.insertRow(i);
    for (var j = 0; j < 4; j++) {
        row.id = data[i][0];
        row.insertCell(j).innerHTML = data[i][j];
    }
}

// builds range slider and event listener
var range = document.createElement('input');
range.type = "range";
range.min = 1;
range.max = data.length - 1;
range.value = 1;
range.addEventListener('input', function () {
    console.log(data[this.value]);
    updateRotation(data[this.value]);
    setActive(this.value);
});

///////////////////////////////////////////////////
//
//       Model loading and manipulation
//
///////////////////////////////////////////////////

// create scene
var scene = new THREE.Scene();
var model_scale = 0.035;
// Load Camera Perspective
var camera = new THREE.PerspectiveCamera(25, window.innerWidth / window.innerHeight, 1, 20000);
camera.position.set(20, 7.5, 20);

// Load a Renderer
var renderer = new THREE.WebGLRenderer({ alpha: false });
renderer.setClearColor('black');
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth * .75, window.innerHeight * .75);
document.getElementById('data-table').appendChild(tab);
document.getElementById('content-container').appendChild(range);
document.getElementById('capsule-container').appendChild(renderer.domElement);

// Load the Orbitcontroller
var controls = new OrbitControls(camera, renderer.domElement);
controls.addEventListener("change", event => {
    console.log(controls.object.position);
});

// Load Light
var ambientLight = new THREE.AmbientLight(0xcccccc);
var directionalLight = new THREE.DirectionalLight(0xffffff);
directionalLight.position.set(0, 1, 1).normalize();
scene.add(ambientLight);
scene.add(directionalLight);

// Loads axis 
// The X axis is red. The Y axis is green. The Z axis is blue.
var axesHelper = new THREE.AxesHelper(7);
scene.add(axesHelper);

// glb Loader
var loader = new GLTFLoader();
var model;
loader.load('./capsule.glb', function (gltf) {
    model = gltf.scene;
    model.scale.set(model_scale, model_scale, model_scale);
    model.position.x = 0;
    model.position.y = 0;
    model.position.z = 0;
    scene.add(model);
    model = gltf.scene;
}, function (xhr) {
    console.log((xhr.loaded / xhr.total * 100) + '% loaded');
}, function (error) {
    console.log('Error: ' + error);
});

// Render and animate model
function animate() {
    render();
    requestAnimationFrame(animate);
}
function render() {
    renderer.render(scene, camera);
}
render();
animate();

// Update model rotation
function updateRotation(angles) {
    // three.js axis of rotation differs from Adafruit_BNO055
    // Adjusted below to match the Adafruit_BNO055 datasheet 
    model.rotation.x = degrees_to_radians(angles[2]);
    model.rotation.y = degrees_to_radians(angles[3]);
    model.rotation.z = degrees_to_radians(angles[1]);
    controls.update();
}

// Sets view to default values
function reset() {
    if (prev != -1) prev.classList.remove("active");
    range.value = 1;
    model.scale.set(model_scale, model_scale, model_scale);
    model.position.x = 0;
    model.position.y = 0;
    model.position.z = 0;
    model.rotation.x = 0;
    model.rotation.y = 0;
    model.rotation.z = 0;
    camera.position.set(20, 7.5, 20);
    controls.update();
}

// Begins stepping through the data points 
// and updates model rotation
var prev = -1;
async function start() {
    console.log("start");
    range.value = 1;
    for (var i = 1; i < range.max + 1; i++) {
        updateRotation(data[i]);
        setActive(i);
        await timer(250); // Sets time delay while playing
    }
}

// Listeners for buttons 
document.getElementById("reset").onclick = reset;
document.getElementById("start").onclick = start;
