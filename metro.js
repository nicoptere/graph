var req = new XMLHttpRequest();
req.onload = function(e)
{
    var json = JSON.parse(e.target.responseText );
    init( json );

};
req.open( 'GET', 'metro.json' );
req.send();




var ctx = document.getElementById("canvas").getContext('2d');
var size = 600;// Math.min( window.innerWidth, window.innerHeight );
var w = ctx.canvas.width = size;
var h = ctx.canvas.height = size;

var lines, graph;
var margin = 25;

//raf: https://github.com/cagosta/requestAnimationFrame/blob/master/app/requestAnimationFrame.js
(function(global) {(function() {if (global.requestAnimationFrame) {return;} if (global.webkitRequestAnimationFrame) {global.requestAnimationFrame = global[ 'webkitRequestAnimationFrame' ]; global.cancelAnimationFrame = global[ 'webkitCancelAnimationFrame' ] || global[ 'webkitCancelRequestAnimationFrame' ];} var lastTime = 0; global.requestAnimationFrame = function(callback) {var currTime = new Date().getTime(); var timeToCall = Math.max(0, 16 - (currTime - lastTime)); var id = global.setTimeout(function() {callback(currTime + timeToCall);}, timeToCall); lastTime = currTime + timeToCall; return id;}; global.cancelAnimationFrame = function(id) {clearTimeout(id);};})(); if (typeof define === 'function') {define(function() {return global.requestAnimationFrame;});}})(window);
function lerp ( t, a, b ){ return a + t * ( b - a ); }
function norm( t, a, b ){return ( t - a ) / ( b - a );}
function map( t, a0, b0, a1, b1 ){ return lerp( norm( t, a0, b0 ), a1, b1 );}

function remap( lat, lng, min, max, margin )
{
    return [       map( lng, min[1], max[1], margin, w-margin ),
        h - map( lat, min[0], max[0], margin, h-margin ) ];
}
function init(json) {

    var key;
    lines = json.lignes;

    var stations = [];
    var min = [Number.POSITIVE_INFINITY,Number.POSITIVE_INFINITY];
    var max = [Number.NEGATIVE_INFINITY,Number.NEGATIVE_INFINITY];
    for( key in json.stations ){

        var obj = json.stations[key];
        if( obj.type == "metro" )
        {
            var o = { id:obj.num, n:obj.nom, lat:parseFloat( obj.lat ), lng:parseFloat( obj.lng ) };
            if(o.lat < min[0])min[0]= o.lat;
            if(o.lng < min[1])min[1]= o.lng;
            if(o.lat > max[0])max[0]= o.lat;
            if(o.lng > max[1])max[1]= o.lng;
            stations.push( o );
        }
    }

    var vertices = [];
    stations.forEach( function(o){

        var p = remap(o.lat, o.lng, min, max, margin );
        o.x = p[0];
        o.y = p[1];
        vertices.push( new Vertex( o ) );
    });

    graph = new Graph( vertices, [] );
    graph.buildDictionary("id");

    for( key in lines )
    {
        lines[ key ].graph = new Graph( vertices, [] );
        lines[ key ].graph.buildDictionary( "id" );
    }

    json.routes.forEach(function(obj){

        if( obj.type == "metro" ){

            var g = lines[ obj.ligne ].graph;
            var arr = obj.arrets;
            for( var i = 0; arr.length - 1; i++ ){
                var s = graph[arr[i] ];
                var e = graph[arr[i+1]];
                if( s == null )return;
                if( e == null )return;
                var edge = new Edge( s,e );
                graph.addEdge( edge );
                g.addEdge( edge );
            }

        }
    });
    graph.flush();

    for( key in lines ) {
        var g = lines[key].graph;
        g.collapse();
        g.buildDictionary("id");
    }

    var ham = new Hammer( ctx.canvas );
    ham.on( 'tap' , onDown );
    ham.on( 'pan' , onMove );

    //window.onmousemove = onMove;
    //window.onmouseup = onUp;
    //window.onmousedown = onDown;

    update();
}
function update()
{
    requestAnimationFrame(update);
    ctx.clearRect(0,0,w,h);

    ctx.globalCompositeOperation = "source-over";
    ctx.strokeStyle = ctx.fillStyle = "#222";
    ctx.fillRect(0,0,w,h);
    for( var key in lines )
    {

        var g = lines[ key ].graph;
        //g.collapse();
        ctx.strokeStyle = ctx.fillStyle = lines[ key ].color;
        utils.renderEdges( g, ctx,  5 );

    }
    ctx.strokeStyle = ctx.fillStyle = "#000";
    utils.renderVertices( graph, ctx,  3 );

    if( root ){
        ctx.strokeStyle = "#F00";
        c( root, 5 );
    }
    if( target ){
        ctx.strokeStyle = "#06C";
        c( target, 5);
    }

    //print station name
    closest = getClosest( mouse, graph );
    ctx.strokeStyle = "#FFF";
    c( closest, 10 );
    ctx.font = "12px Verdana";
    var tw = ctx.measureText(closest.n).width;
    ctx.fillStyle = "#FFF";
    ctx.fillRect( closest.x - tw / 2 - 5, closest.y - 12 - 15, tw + 10, 20 );
    ctx.fillStyle = "#000";
    ctx.fillText( closest.n, closest.x - tw / 2, closest.y - 12 );

    //trace shortest path
    renderPath();


}

function renderPath()
{

    ctx.strokeStyle = "#888";
    ctx.lineWidth = 24;
    ctx.beginPath();
    ctx.moveTo( margin, h - margin  );
    ctx.lineTo( w-margin, h - margin  );
    ctx.stroke();
    ctx.strokeStyle = "#222";
    ctx.lineWidth = 20;
    ctx.beginPath();
    ctx.moveTo( margin, h - margin  );
    ctx.lineTo( w-margin, h - margin  );
    ctx.stroke();

    if( path == null )return;

    var t = Math.sin(Date.now() * 0.0005 ) * .5 + .5;
    ctx.globalAlpha = 1;
    ctx.globalCompositeOperation = "lighten";
    ctx.strokeStyle = "#F00";
    ctx.lineWidth = 12;
    ctx.beginPath();
    path.forEach( function( v ){    ctx.lineTo(v.x, v.y) });
    ctx.stroke();
    ctx.globalAlpha = 1;
    ctx.globalCompositeOperation = "source-over";
    ctx.lineWidth = 2;
    var p = getPositionAt(path, t);
    c(p, 5);
    ctx.strokeStyle = "#000";
    c(p, 2);

    //colors
    path.forEach( function(c,i,a ) {
        if( i < a.length - 1 )
        {
            var n = a[i+1];
            var cl = [];
            var nl = [];
            for( var key in lines ){
                var l = lines[key];
                if( l.graph.vertices.indexOf( c ) != -1
                    &&  l.graph.vertices.indexOf( n ) != -1 )ctx.strokeStyle = ctx.fillStyle = l.color;
            }

            var cpx = map( i, 0, a.length-1, margin, w-margin );
            var npx = map( i+1, 0, a.length-1, margin, w-margin );

            ctx.lineWidth = 20;
            ctx.beginPath();
            ctx.moveTo(cpx, h - margin );
            ctx.lineTo(npx, h - margin );
            ctx.stroke();

        }
        cpx = map( i, 0, a.length-1, margin, w-margin );
        ctx.lineWidth = .5;
        ctx.fillStyle = "#000";
        ctx.beginPath();
        ctx.arc( cpx, h - margin , 3, 0, Math.PI * 2 );
        ctx.fill();
    });
}

var id = 0, root, target, path, path;
var closest = {x:0, y:0};
var mouse = {x:0, y:0};
var mouseDown = false;
function c(p,r){
    ctx.beginPath();
    ctx.arc(p.x, p.y, r||5, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(p.x, p.y, 1, 0, Math.PI * 2);
    ctx.stroke();
}
function getClosest( p, g )
{
    var min, dist, minDist = Number.POSITIVE_INFINITY;
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

function onMove(e) {
    var r = e.target.getBoundingClientRect();
    var x = e.center.x;
    var y = e.center.y;
    mouse.x = x;
    mouse.y = y;
};

function onUp(e){
    mouseDown = false;
};

function onDown(e){

    mouseDown = true;
    var x = e.center.x;
    var y = e.center.y;
    mouse.x = x;
    mouse.y = y;

    if( id %2 == 0 )
    {
        root = getClosest( mouse, graph );
        path = null;
        target = null;

    }else{

        target = getClosest( mouse, graph );
        if( target == root )return;
        utils.dijkstra.init( graph, root, utils.length2d );
        path = utils.dijkstra.getShortestPath(target);


    }
    id++;
};
