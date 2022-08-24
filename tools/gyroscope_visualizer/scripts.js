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

var prev = -1;
function setActive(i) {
    if (prev != -1) {
        prev.classList.remove("active");
    }
    prev = document.getElementById(interpolated[i][0]);
    document.getElementById(interpolated[i][0]).classList.add("active");
}

const lerp = (x, y, a) => x * (1 - a) + y * a;

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

var interpolated = [];
interpolated.push(data[0]);
for (var i = 1; i < data.length - 1; i++) {
    interpolated.push(data[i]);
    // var avg_t = (data[i][0] + data[i + 1][0]) / 2;
    // var avg_x = (data[i][1] + data[i + 1][1]) / 2;
    // var avg_y = (data[i][2] + data[i + 1][2]) / 2;
    // var avg_z = (data[i][3] + data[i + 1][3]) / 2;

    var avg_t = lerp(data[i][0], data[i + 1][0], 0.5);
    var avg_x = lerp(data[i][1], data[i + 1][1], 0.5);
    var avg_y = lerp(data[i][2], data[i + 1][2], 0.5);
    var avg_z = lerp(data[i][3], data[i + 1][3], 0.5);
    interpolated.push([avg_t.toPrecision(4), avg_x.toPrecision(4), avg_y.toPrecision(4), avg_z.toPrecision(4)]);
}
interpolated.push(data[data.length - 1]);

// builds data table
var tab = document.createElement("table");
tab.id = "data";
for (var i = 0; i < interpolated.length; i++) {
    var row = tab.insertRow(i);
    for (var j = 0; j < 4; j++) {
        row.id = interpolated[i][0];
        row.insertCell(j).innerHTML = interpolated[i][j];
    }
}

// builds range slider and event listener
// var range = document.createElement('input');
// range.type = "range";
// range.min = 1;
// range.max = data.length - 1;
// range.value = 1;
// range.addEventListener('input', function () {
//     console.log(data[this.value]);
//     updateRotation(data[this.value]);
//     setActive(this.value);
// });

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
// document.getElementById('content-container').appendChild(range);
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

// Handles model rotation
var x_rot = 0;
var y_rot = 0;
var z_rot = 0;
var x_mult = 0;
var y_mult = 0;
var z_mult = 0;
function rotate(x_mult, y_mult, z_mult) {
    console.log("I am rotate");
    console.log([x_rot, y_rot, z_rot]);
    model.rotation.x = degrees_to_radians(x_rot);
    model.rotation.y = degrees_to_radians(y_rot);
    model.rotation.z = degrees_to_radians(z_rot);
    x_rot += .1 * x_mult;
    y_rot += .1 * y_mult;
    z_rot += .1 * z_mult;
    controls.update();
}

// Update model rotation
function updateRotation(angles) {
    // three.js axis of rotation differs from Adafruit_BNO055
    // Adjusted below to match the Adafruit_BNO055 datasheet 
    x_mult = angles[2];
    y_mult = angles[3];
    z_mult = angles[1];
    controls.update();
}

// Sets view to default values
function reset() {
    if (prev != -1) prev.classList.remove("active");
    // range.value = 1;
    model.scale.set(model_scale, model_scale, model_scale);
    model.position.x = 0;
    model.position.y = 0;
    model.position.z = 0;
    model.rotation.x = 0;
    model.rotation.y = 0;
    model.rotation.z = 0;
    x_rot = 0;
    y_rot = 0;
    z_rot = 0;
    x_mult = 0;
    y_mult = 0;
    z_mult = 0;
    camera.position.set(20, 7.5, 20);
    counter.innerHTML = 0;
    controls.update();
}

// Begins stepping through the data points 
// and updates model rotation
var counter = document.getElementById("timer");
counter.innerHTML = 0;
async function start() {
    console.log("start");
    var j = 1;
    for (var i = 1; i < (Math.ceil(interpolated[interpolated.length - 1][0]) + 3) * 10; i++) {
        counter.innerHTML = (i * .1).toPrecision(3);
        rotate(x_mult, y_mult, z_mult);
        if (i == Math.ceil(interpolated[j][0]) * 10) {
            console.log("UPDATING");
            setActive(j);
            updateRotation(interpolated[j]);
            j++;
            if (j > interpolated.length - 1) j = interpolated.length - 1;
        }
        await timer(100); // Sets time delay while playing
    }
}

// Listeners for buttons 
document.getElementById("reset").onclick = reset;
document.getElementById("start").onclick = start;
