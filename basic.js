/**
 * Created by nico on 16/01/2016.
 */

var ctx = document.getElementById("canvas").getContext('2d');
var w = ctx.canvas.width = 600;
var h = ctx.canvas.height = 600;


function reset()
{
    ctx.clearRect( 0,0,w,h );
    var vertices = [];
    for( var i = 0; i < 100; i++ ){
        vertices.push( new Vertex( {x:-50+Math.random() * ( w + 100 ), y:-50 + Math.random() * ( h+100) } ) );
    }

    var g = new Graph( vertices );
    utils.triangulate( g );

    var g0 = utils.triangulateCenters( g );
    g0.removeDuplicateEdges();

    ctx.strokeStyle ="#AAA";
    utils.renderEdges(g, ctx,.5);
    ctx.strokeStyle ="#000";
    utils.renderVertices(g0, ctx,2);

    var mst = utils.mst.compute( g0, utils.length2d );
    var g1 = new Graph( g0.vertices, mst );
    g1.flush();
    ctx.strokeStyle = "blue";
    utils.renderEdges(g1, ctx,.5);
}

var ham = new Hammer( ctx.canvas );
ham.on('tap', reset );
reset();


/*
//dijkstra path finding
var root = g1.vertices[parseInt( g1.vertices.length * Math.random() ) ];
var target = g1.vertices[parseInt( g1.vertices.length * Math.random() ) ];
utils.dijkstra.init( g1, root );
var path = utils.dijkstra.getShortestPath( target );


function c(p,r){
    ctx.beginPath();
    ctx.arc(p.x, p.y, r||5, 0, Math.PI * 2);
    ctx.fill();
}
ctx.lineWidth = 1;
ctx.strokeStyle = ctx.fillStyle ="blue";
c( root);
ctx.strokeStyle = ctx.fillStyle ="red";
c( target);
ctx.lineWidth = 3;
ctx.beginPath();
path.forEach( function( v ){    ctx.lineTo(v.x, v.y) });
ctx.stroke();

//*/