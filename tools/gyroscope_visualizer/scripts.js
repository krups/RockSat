import * as THREE from 'three';
import { GLTFLoader } from './three/examples/jsm/loaders/GLTFLoader.js';
import { OrbitControls } from './three/examples/jsm/controls/OrbitControls.js';

var scene = new THREE.Scene();

// Load Camera Perspektive
var camera = new THREE.PerspectiveCamera(25, window.innerWidth / window.innerHeight, 1, 20000);
camera.position.set(1, 1, 20);

// Load a Renderer
var renderer = new THREE.WebGLRenderer({ alpha: false });
renderer.setClearColor(0xC5C5C3);
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth * .75, window.innerHeight * .75);
document.getElementById('capsule-container').appendChild(renderer.domElement);

// Load the Orbitcontroller
var controls = new OrbitControls(camera, renderer.domElement);

// Load Light
var ambientLight = new THREE.AmbientLight(0xcccccc);
scene.add(ambientLight);

var directionalLight = new THREE.DirectionalLight(0xffffff);
directionalLight.position.set(0, 1, 1).normalize();
scene.add(directionalLight);

// glb Loader
var loader = new GLTFLoader();
loader.load('./capsule.glb', function (gltf) {
    var object = gltf.scene;
    gltf.scene.scale.set(0.03, 0.03, 0.03);
    gltf.scene.position.x = 0;
    gltf.scene.position.y = 0;
    gltf.scene.position.z = 0;

    scene.add(gltf.scene);
});

function animate() {
    render();
    requestAnimationFrame(animate);
}

function render() {
    renderer.render(scene, camera);
}

render();
animate();