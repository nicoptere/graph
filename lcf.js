


var ctx = document.getElementById("canvas").getContext('2d');
var w = ctx.canvas.width = window.innerWidth;
var h = ctx.canvas.height = window.innerHeight;
ctx.lineCap = 'round';

var graph;
var restLength = Math.min( w,h )/2;
var sel = document.getElementById('presets');
var des = document.getElementById('current');
sel.onchange = function(e)
{
    des.innerText = e.target.selectedOptions[0].text + " " + e.target.selectedOptions[0].value;
    graph = utils.LCF.build( e.target.selectedOptions[0].value, restLength );
};

graph = utils.LCF.build( sel.selectedOptions[0].value, restLength );
des.innerText = sel.selectedOptions[0].text + " " + sel.selectedOptions[0].value;

var mouse = 0;
var ham = new Hammer(ctx.canvas);
ham.on( 'pan', function(e) { mouse = e.center.x / w - .5; });

var RAD = Math.PI / 180;
var mat = mat4.create();
var per = mat4.create();
mat4.perspective(per, 45*RAD, w/h,.1, 10000);
var vec = vec3.create();
function update(){

    requestAnimationFrame( update );

    ctx.restore();
    ctx.clearRect( 0,0,w,h );
    ctx.save();
    ctx.translate(w/2, h/2);

    mat4.identity(mat);
    mat4.rotate(mat, mat, mouse * RAD, [0,1,0] );

    graph.vertices.forEach(function( a ) {
        //attraction
        a.neighbours.forEach(function( b ) {

            var dx = b.x - a.x;
            var dy = b.y - a.y;
            var dz = b.z - a.z;
            var d = Math.sqrt(dx * dx + dy * dy + dz * dz);
            var diff = ( restLength / a.neighbours.length  ) - d;
            var offx = (diff * dx / d) * .5;
            var offy = (diff * dy / d) * .5;
            var offz = (diff * dz / d) * .5;
            a.x -= offx;
            a.y -= offy;
            a.z -= offz;
            b.x += offx;
            b.y += offy;
            b.z += offz;

        });
        //repulsion
        graph.vertices.forEach(function( b ) {

            if( a == b ) return;
            var dx = b.x - a.x;
            var dy = b.y - a.y;
            var dz = b.z - a.z;
            var d = Math.sqrt(dx * dx + dy * dy + dz * dz);
            var diff = ( restLength ) - d;
            var offx = (diff * dx / d) * .01;
            var offy = (diff * dy / d) * .01;
            var offz = (diff * dz / d) * .01;
            a.x -= offx;
            a.y -= offy;
            a.z -= offz;
            b.x += offx;
            b.y += offy;
            b.z += offz;

        });
    });

    //projection
    graph.vertices.forEach(function(v){

        vec[0] = v.x;
        vec[1] = v.y;
        vec[2] = v.z;
        vec3.transformMat4(vec, vec, mat );
        v.x = vec[0];
        v.y = vec[1];
        v.z = vec[2];
    });


    render();

}

function render(){



    ctx.lineWidth = 1.5;
    graph.edges.forEach( function(e){

        ctx.lineWidth = ( (restLength * .5 + ( e.v0.z+ e.v1.z) *.5) / restLength ) * .5 + .1;
        ctx.beginPath();
        ctx.moveTo(e.v0.x, e.v0.y );
        ctx.lineTo(e.v1.x, e.v1.y );
        ctx.stroke();

    });

    ctx.lineWidth = 5;
    ctx.strokeStyle = "#000";
    graph.vertices.forEach( function(v){
        ctx.lineWidth = ( (restLength * .5 + v.z) / restLength ) * 5 + 1;
        ctx.beginPath();
        ctx.moveTo(v.x, v.y );
        ctx.lineTo(v.x + 1, v.y );
        ctx.stroke();
    });
}
update();

