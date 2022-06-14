import * as THREE from "https://cdn.skypack.dev/three@0.132.2";
  
import { OrbitControls } from "https://cdn.skypack.dev/three@0.132.2/examples/jsm/controls/OrbitControls.js";
import { STLLoader } from 'https://cdn.skypack.dev/three@0.132.2/examples/jsm/loaders/STLLoader.js';
import Stats from 'https://cdn.skypack.dev/three@0.132.2/examples/jsm/libs/stats.module.js';

const scene = new THREE.Scene();
scene.add(new THREE.AxesHelper(5));

const light1 = new THREE.PointLight( 0xffffff, 1);
light1.position.set( 200, 200, 200 );
scene.add(light1);

const light2 = new THREE.PointLight( 0xffffff, 1);
light2.position.set( -200, -200, -200 );
scene.add(light2);

const light3 = new THREE.PointLight( 0xffffff, 1);
light1.position.set( 1200, 1200, 1200 );
scene.add(light3);

const light4 = new THREE.PointLight( 0xffffff, 1);
light2.position.set( -1200, -1200, -1200 );
scene.add(light4);

const camera = new THREE.PerspectiveCamera(
    45,
    window.innerWidth / window.innerHeight,
    1,
    3000
);
camera.position.x = 100;
camera.position.y = 600;
camera.position.z = -500;

const renderer = new THREE.WebGLRenderer();
renderer.outputEncoding = THREE.sRGBEncoding;
renderer.setSize(window.innerWidth, window.innerHeight);
document.getElementById("robot").appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

const material = new THREE.MeshPhysicalMaterial({
    color: 0xb2ffc8,
    envMap: null,
    metalness: 0.4,
    roughness: 0.6,
    opacity: 1.0,
    transparent: false,
    transmission: 0.99,
    clearcoat: 1.0,
    clearcoatRoughness: 0.25
});

controls.target.set(0, 0, 0);

const loader = new STLLoader();
loader.load(
    'robot.stl',
    function (geometry) {
        geometry.translate(0, 0, -200);
        const mesh = new THREE.Mesh(geometry, material)
        scene.add(mesh)
    },
    (xhr) => {
        console.log((xhr.loaded / xhr.total) * 100 + '% loaded')
    },
    (error) => {
        console.log(error)
    }
);

window.addEventListener('resize', onWindowResize, false);
function onWindowResize() {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth / 2, window.innerHeight / 2);
    render();
}

const stats = Stats();
document.getElementById("robot").appendChild(stats.dom);

function animate() {
    requestAnimationFrame(animate);

    controls.update();

    render();

    stats.update();
}

function render() {
    renderer.render(scene, camera);
}

animate();