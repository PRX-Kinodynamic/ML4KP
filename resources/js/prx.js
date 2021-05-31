function CapsuleBufferGeometry( radiusTop, radiusBottom, height, radialSegments, heightSegments, capsTopSegments, capsBottomSegments, thetaStart, thetaLength ) {

  THREE.BufferGeometry.call( this );

  this.type = 'CapsuleBufferGeometry';

  this.parameters = {
    radiusTop: radiusTop,
    radiusBottom: radiusBottom,
    height: height,
    radialSegments: radialSegments,
    heightSegments: heightSegments,
    thetaStart: thetaStart,
    thetaLength: thetaLength
  };

  var scope = this;

  radiusTop = radiusTop !== undefined ? radiusTop : 1;
  radiusBottom = radiusBottom !== undefined ? radiusBottom : 1;
  height = height !== undefined ? height : 2;

  radialSegments = Math.floor( radialSegments ) || 8;
  heightSegments = Math.floor( heightSegments ) || 1;
    capsTopSegments = Math.floor( capsTopSegments ) || 2;
    capsBottomSegments = Math.floor( capsBottomSegments ) || 2;

  thetaStart = thetaStart !== undefined ? thetaStart : 0.0;
  thetaLength = thetaLength !== undefined ? thetaLength : 2.0 * Math.PI;

    // Alpha is the angle such that Math.PI/2 - alpha is the cone part angle.
    var alpha = Math.acos((radiusBottom-radiusTop)/height);
    var eqRadii = (radiusTop-radiusBottom === 0);

  var vertexCount = calculateVertexCount();
  var indexCount = calculateIndexCount();

  // buffers
  var indices = new THREE.BufferAttribute( new ( indexCount > 65535 ? Uint32Array : Uint16Array )( indexCount ), 1 );
  var vertices = new THREE.BufferAttribute( new Float32Array( vertexCount * 3 ), 3 );
  var normals = new THREE.BufferAttribute( new Float32Array( vertexCount * 3 ), 3 );
  var uvs = new THREE.BufferAttribute( new Float32Array( vertexCount * 2 ), 2 );

  // helper variables

  var index = 0,
      indexOffset = 0,
      indexArray = [],
      halfHeight = height / 2;

  // generate geometry

  generateTorso();

  // build geometry

  this.setIndex( indices );
  this.addAttribute( 'position', vertices );
  this.addAttribute( 'normal', normals );
  this.addAttribute( 'uv', uvs );

  // helper functions

    function calculateVertexCount(){
        var count = ( radialSegments + 1 ) * ( heightSegments + 1 + capsBottomSegments + capsTopSegments);
        return count;
    }

  function calculateIndexCount() {
    var count = radialSegments * (heightSegments + capsBottomSegments + capsTopSegments) * 2 * 3;
    return count;
  }

  function generateTorso() {

    var x, y;
    var normal = new THREE.Vector3();
    var vertex = new THREE.Vector3();

        var cosAlpha = Math.cos(alpha);
        var sinAlpha = Math.sin(alpha);

        var cone_length =
            new THREE.Vector2(
                radiusTop*sinAlpha,
                halfHeight+radiusTop*cosAlpha
                ).sub(new THREE.Vector2(
                    radiusBottom*sinAlpha,
                    -halfHeight+radiusBottom*cosAlpha
                )
            ).length();

        // Total length for v texture coord
        var vl = radiusTop*alpha
                 + cone_length
                 + radiusBottom*(Math.PI/2-alpha);

    var groupCount = 0;

    // generate vertices, normals and uvs

        var v = 0;
        for( y = 0; y <= capsTopSegments; y++ ) {

            var indexRow = [];

            var a = Math.PI/2 - alpha*(y / capsTopSegments);

            v += radiusTop*alpha/capsTopSegments;

            var cosA = Math.cos(a);
            var sinA = Math.sin(a);

            // calculate the radius of the current row
      var radius = cosA*radiusTop;

            for ( x = 0; x <= radialSegments; x ++ ) {

        var u = x / radialSegments;

        var theta = u * thetaLength + thetaStart;

        var sinTheta = Math.sin( theta );
        var cosTheta = Math.cos( theta );

        // vertex
        vertex.x = radius * sinTheta;
        vertex.y = halfHeight + sinA*radiusTop;
        vertex.z = radius * cosTheta;
        vertices.setXYZ( index, vertex.x, vertex.y, vertex.z );

        // normal
        normal.set( cosA*sinTheta, sinA, cosA*cosTheta );
        normals.setXYZ( index, normal.x, normal.y, normal.z );

        // uv
        uvs.setXY( index, u, 1 - v/vl );

        // save index of vertex in respective row
        indexRow.push( index );

        // increase index
        index ++;

      }

            // now save vertices of the row in our index array
      indexArray.push( indexRow );

        }

        var cone_height = height + cosAlpha*radiusTop - cosAlpha*radiusBottom;
        var slope = sinAlpha * ( radiusBottom - radiusTop ) / cone_height;
    for ( y = 1; y <= heightSegments; y++ ) {

      var indexRow = [];

      v += cone_length/heightSegments;

      // calculate the radius of the current row
      var radius = sinAlpha * ( y * ( radiusBottom - radiusTop ) / heightSegments + radiusTop);

      for ( x = 0; x <= radialSegments; x ++ ) {

        var u = x / radialSegments;

        var theta = u * thetaLength + thetaStart;

        var sinTheta = Math.sin( theta );
        var cosTheta = Math.cos( theta );

        // vertex
        vertex.x = radius * sinTheta;
        vertex.y = halfHeight + cosAlpha*radiusTop - y * cone_height / heightSegments;
        vertex.z = radius * cosTheta;
        vertices.setXYZ( index, vertex.x, vertex.y, vertex.z );

        // normal
        normal.set( sinTheta, slope, cosTheta ).normalize();
        normals.setXYZ( index, normal.x, normal.y, normal.z );

        // uv
        uvs.setXY( index, u, 1 - v/vl );

        // save index of vertex in respective row
        indexRow.push( index );

        // increase index
        index ++;

      }

      // now save vertices of the row in our index array
      indexArray.push( indexRow );

    }

        for( y = 1; y <= capsBottomSegments; y++ ) {

            var indexRow = [];

            var a = (Math.PI/2 - alpha) - (Math.PI - alpha)*( y / capsBottomSegments);

            v += radiusBottom*alpha/capsBottomSegments;

            var cosA = Math.cos(a);
            var sinA = Math.sin(a);

            // calculate the radius of the current row
      var radius = cosA*radiusBottom;

            for ( x = 0; x <= radialSegments; x ++ ) {

        var u = x / radialSegments;

        var theta = u * thetaLength + thetaStart;

        var sinTheta = Math.sin( theta );
        var cosTheta = Math.cos( theta );

        // vertex
        vertex.x = radius * sinTheta;
        vertex.y = -halfHeight + sinA*radiusBottom;;
        vertex.z = radius * cosTheta;
        vertices.setXYZ( index, vertex.x, vertex.y, vertex.z );

        // normal
        normal.set( cosA*sinTheta, sinA, cosA*cosTheta );
        normals.setXYZ( index, normal.x, normal.y, normal.z );

        // uv
        uvs.setXY( index, u, 1 - v/vl );

        // save index of vertex in respective row
        indexRow.push( index );

        // increase index
        index ++;

      }

            // now save vertices of the row in our index array
      indexArray.push( indexRow );

        }

    // generate indices

    for ( x = 0; x < radialSegments; x ++ ) {

      for ( y = 0; y < capsTopSegments + heightSegments + capsBottomSegments; y ++ ) {

        // we use the index array to access the correct indices
        var i1 = indexArray[ y ][ x ];
        var i2 = indexArray[ y + 1 ][ x ];
        var i3 = indexArray[ y + 1 ][ x + 1 ];
        var i4 = indexArray[ y ][ x + 1 ];

        // face one
        indices.setX( indexOffset, i1 ); indexOffset ++;
        indices.setX( indexOffset, i2 ); indexOffset ++;
        indices.setX( indexOffset, i4 ); indexOffset ++;

        // face two
        indices.setX( indexOffset, i2 ); indexOffset ++;
        indices.setX( indexOffset, i3 ); indexOffset ++;
        indices.setX( indexOffset, i4 ); indexOffset ++;

      }

    }

  }

}

CapsuleBufferGeometry.prototype = Object.create( THREE.BufferGeometry.prototype );
CapsuleBufferGeometry.prototype.constructor = CapsuleBufferGeometry;

var scene, camera, renderer;
var mixers = [];
var play_animation = false;
image_width =600;
image_height=600;

scene = new THREE.Scene();
camera = new THREE.PerspectiveCamera( 45, image_width / image_height, 0.1, 1000 );
renderer = new THREE.WebGLRenderer();
renderer.setSize( image_width, image_height );
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFShadowMap;
document.body.appendChild( renderer.domElement );
camera.position.x = 0;
camera.position.y = -5;
camera.position.z = 20;
camera.up.set( 0, 0, 1 );
camera.lookAt( 0, 0, 0 );
var axesHelper = new THREE.AxesHelper( 10 );
axesHelper.position.z+=.6
// scene.add( axesHelper );

// var gridHelper = new THREE.GridHelper( 100, 100/5, 0x0000ff, 0xb8b8b8 );
//        gridHelper.rotation.x=Math.PI / 2;
//        gridHelper.position.z+=.04
//        scene.add( gridHelper );

var geometry = new THREE.PlaneBufferGeometry( 200, 200 );
        var material = new THREE.MeshPhongMaterial( { color: 0xbbbbbb } );
        var background = new THREE.Mesh( geometry, material );
        background.receiveShadow = true;
        background.position.set( 0, 0, -.02 );
        background.quaternion=new THREE.Quaternion(.707,0,0,.707);
        scene.add( background );

controls = new THREE.MapControls( camera, renderer.domElement );
controls.enableDamping = true;
controls.dampingFactor = 0.25;
controls.screenSpacePanning = true;
controls.minDistance = 0;
controls.maxDistance = 200;
controls.maxPolarAngle = Math.PI / 2;
var light = new THREE.DirectionalLight( 0xffffff );
light.position.set( 100, 100, 200 );
// light.castShadow = true;
// light.shadow.camera.near = 1;
// light.shadow.camera.far = 350;
// light.shadow.camera.right = 150;
// light.shadow.camera.left = - 150;
// light.shadow.camera.top  = 150;
// light.shadow.camera.bottom = - 150;
// light.shadow.mapSize.width = 4000;
// light.shadow.mapSize.height = 4000;
scene.add( light );
var light = new THREE.DirectionalLight( 0xaaaaaa );
light.position.set( -50, -50, 50 );
light.castShadow = true;
light.shadow.camera.near = 1;
light.shadow.camera.far = 350;
light.shadow.camera.right = 150;
light.shadow.camera.left = - 150;
light.shadow.camera.top = 150;
light.shadow.camera.bottom = - 150;
light.shadow.mapSize.width = 4000;
light.shadow.mapSize.height = 4000;
scene.add( light );
// var light = new THREE.DirectionalLight( 0x444444 );
// light.position.set( -50, 50, 50 );
// light.castShadow = true;
// scene.add( light );
var light = new THREE.AmbientLight( 0x222222 );
scene.add( light );

var clock = new THREE.Clock();
document.getElementById("shot").addEventListener('click', takeScreenshot);
document.getElementById("bt_play").addEventListener('click', start_stop_animation);

function start_stop_animation() {
  if (play_animation)
  {
    play_animation = false;
    clock.stop();
  }
  else
  {
    play_animation = true;
    clock.start();
  }
}

function takeScreenshot() {

    // open in new window like this
    //
    // var w = window.open('', '');
    // w.document.title = "Screenshot";
    // //w.document.body.style.backgroundColor = "red";
    // var img = new Image();
    // // Without 'preserveDrawingBuffer' set to true, we must render now
    // renderer.render(scene, camera);
    // img.src = renderer.domElement.toDataURL();
    // w.document.body.appendChild(img);  

    // download file like this.
    //
    var a = document.createElement('a');
    // Without 'preserveDrawingBuffer' set to true, we must render now
    renderer.render(scene, camera);
    a.href = renderer.domElement.toDataURL().replace("image/png", "image/octet-stream");
    a.download = 'screenshot.png'
    a.click();
    
}

function animate() 
{
  requestAnimationFrame( animate );
  controls.update();
  if (play_animation)
  {
    var delta = clock.getDelta();
    for (i = 0; i < mixers.length; i++) {
      mixers[i].update(delta)
    }
  }
  else
  {
    var slider = document.getElementById("time_slider");
    // alert(clip.duration);
    // alert(( slider.value / slider.max) );
    // alert(( slider.value / slider.max) * clip.duration);
    for (i = 0; i < mixers.length; i++) {
      mixers[i].setTime( ( slider.value / slider.max) * clip.duration);
    }
  }
  renderer.render( scene, camera );
}