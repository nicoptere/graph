
var ctx = document.getElementById("canvas").getContext('2d');
var w = ctx.canvas.width = 600;//window.innerWidth;
var h = ctx.canvas.height = 600;//window.innerHeight;
ctx.translate(w/2, h/2);


var graph, mesh;
function build( vertices, indices ){

    var vs = [];
    var cs = [];

    var c = {x:0, y:0, z:0};
    for( var i = 0; i < vertices.length; i+=3 ){
        var v = new Vertex({x:vertices[i], y:-vertices[i+1], z:vertices[i+2] });
        c.x += v.x;
        c.y += v.y;
        c.z += v.z;
        vs.push( v );
    }

    c.x/=vs.length;
    c.y/=vs.length;
    c.z/=vs.length;

    mat4.identity(mat);
    mat4.rotateX( mat, mat, Math.PI / 180 * -45 );
    vs.forEach(function(v){

        v.x -= c.x;
        v.y -= c.y;
        v.z -= c.z;

        vec[0] = v.x;
        vec[1] = v.y;
        vec[2] = v.z;
        vec3.transformMat4(vec, vec, mat  );
        v.x = vec[0];
        v.y = vec[1];
        v.z = vec[2];

        v.x *= scale;
        v.y *= scale;
        v.z *= scale;

    });

    origins = vs.concat();

    var es = [];
    for( i = 0; i < indices.length; i+=3 ){

        var i0 = indices[ i ];
        var i1 = indices[ i+1 ];
        var i2 = indices[ i+2 ];
        es.push( new Edge( vs[ i0 ], vs[ i1 ] ) );
        es.push( new Edge( vs[ i1 ], vs[ i2 ] ) );
        es.push( new Edge( vs[ i2 ], vs[ i0 ] ) );

        c = {
            x:( vs[ i0].x +  vs[ i1].x +  vs[ i2].x ) / 3,
            y:( vs[ i0].y +  vs[ i1].y +  vs[ i2].y ) / 3,
            z:( vs[ i0].z +  vs[ i1].z +  vs[ i2].z ) / 3
        };
        cs.push( new Vertex( c ) );


    }

    mesh = new Graph( vs,es );
    mesh.removeDuplicateEdges();

    graph = new Graph( mesh.vertices, utils.mst.compute( mesh, utils.length3d ) );
    graph.flush();
    ends = graph.getEnds();

    update();

}

var mat = mat4.create();
var Y = vec3.create();
Y[1] = 1;
var vec = vec3.create();

var id = 0, root, target, mst_path, path;
var mouse = {x:0, y:0};
function update(){
    requestAnimationFrame(update);

    mat4.identity(mat);
    mat4.rotate(mat, mat,  -mouse.x * Math.PI / 180, Y );

    mesh.vertices.forEach( function(v, i){
        vec[0] = v.x;
        vec[1] = v.y;
        vec[2] = v.z;
        vec3.transformMat4(vec, vec, mat  );
        v.x = vec[0];
        v.y = vec[1];
        v.z = vec[2];
    } );

    graph.vertices.forEach( function(v, i){
        vec[0] = v.x;
        vec[1] = v.y;
        vec[2] = v.z;
        vec3.transformMat4(vec, vec, mat  );
        v.x = vec[0];
        v.y = vec[1];
        v.z = vec[2];
    });


    ctx.clearRect( -w/2, -h/2, w, h );

    ctx.lineJoin = "round";
    ctx.strokeStyle = "#BBB";
    renderEdges( mesh, ctx, .5, true );

    if( mst_path ){
        ctx.strokeStyle = "#FC0";
        ctx.lineWidth = 5;
        ctx.beginPath();
        mst_path.forEach( function( v ){    ctx.lineTo(v.x, v.y) });
        ctx.stroke();var t = Math.sin(Date.now() * 0.0005 ) * .5 + .5;
        var p = getPositionAt(mst_path, t);
        c(p, 5);
        ctx.strokeStyle = "#000";
        c(p, 2);
    }

    ctx.strokeStyle = "#666";
    renderVertices( mesh, ctx, 3 );
    renderEdges( graph, ctx, 1 );

    if( root ){
        ctx.strokeStyle = "#C06";
        c( root, 5 );
    }
    if( target ){
        ctx.strokeStyle = "#06C";
        c( target, 5 );
    }
}

function c(p,r){
    ctx.beginPath();
    ctx.arc(p.x, p.y, r||5, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(p.x, p.y, 1, 0, Math.PI * 2);
    ctx.stroke();
}

function renderVertices( graph, ctx, pointSize, lineSize ) {
    ctx.lineCap = "round";
    ctx.lineWidth = pointSize || 3;
    ctx.beginPath();
    if( pointSize != 0 ){
        graph.vertices.forEach(function(v){
            ctx.moveTo( v.x, v.y );
            ctx.lineTo(v.x +.5, v.y);
        })
    }
    ctx.stroke();
}

function renderEdges(graph, ctx, lineSize, front )
{
    ctx.lineWidth = lineSize || .5;
    ctx.globalAlpha = 1;
    ctx.beginPath();
    graph.edges.forEach( function( e ){
        if(Boolean( front ) && e.v0.z<0)return;
        if(Boolean( front ) && e.v1.z<0)return;
        ctx.moveTo(e.v0.x, e.v0.y );
        ctx.lineTo(e.v1.x, e.v1.y );
    });
    ctx.stroke();
};

function getClosest( p, g )
{
    var min, dist, minDist = Math.pow( 2, 53 );
    g.vertices.forEach(function(v)
    {
        var dx = v.x - p.x;
        var dy = v.y - p.y;
        dist = ( dx*dx + dy*dy );
        if( dist < minDist ){
            minDist = dist;
            min = v;
        }
    });
    return min;
}


function lerp ( t, a, b ){ return a + t * ( b - a ); }
function norm( t, a, b ){return ( t - a ) / ( b - a );}
function map( t, a0, b0, a1, b1 ){ return lerp( norm( t, a0, b0 ), a1, b1 );}
function getPositionAt( points, t ) {

    var length = points.length-1;
    var i0 = Math.floor( length * t );
    i0 = i0 < length - 1 ? i0 : length - 1;
    var i1 = Math.min( i0 + 1, length );

    var delta = 1 / length;
    var nt =  ( t - ( i0 * delta ) ) / delta;
    return {x:lerp( nt, points[i0].x, points[i1].x ),
        y:lerp( nt, points[i0].y, points[i1].y ) };
}

var mouseDown = false;

function onMove(e) {
    var r = e.target.getBoundingClientRect();
    var x = e.center.x;// - r.left;
    var y = e.center.y;// - r.top ;
    mouse.x = (x-w/2) / w;
    mouse.y = (y-h/2) / h;

};

function onDown(e){

    mouseDown = true;
    var r = e.target.getBoundingClientRect();
    var x = e.clientX - r.left - r.width / 2;
    var y = e.clientY - r.top - r.height / 2;
    e.center.x -= 300;
    e.center.y -= 300;
    if( id %2 == 0 )
    {
        root = getClosest(e.center, mesh );
        mst_path = null;
        target = null;

    }else{

        target = getClosest( e.center, mesh );
        if( root == target ) return;
        utils.dijkstra.init( graph, root, utils.length3d );
        mst_path = utils.dijkstra.getShortestPath(target);
    }
    id++;
};
var ham = new Hammer( ctx.canvas );
ham.on('pan', onMove);
ham.on('tap', onDown);

scale = 1.35;
objLoader.load( "obj/crab.obj", build );
