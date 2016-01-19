var objLoader = function(exports){

    exports.load = function(url, cb ){

        var req = new XMLHttpRequest();
        req.onload = function( e ){
            parse(e.target.responseText, cb);
        };
        req.open("GET", url );
        req.send();
    };

    function parse(str, cb){

        console.time( 'vertices' );
        var vertices = [];
        var vs = str.match(/v\s.*/g);
        vs.map( function( v )
        {
            var st = v.replace( /v\s+/, '').split( /\s/ );
            vertices.push( parseFloat(st[0]), parseFloat(st[1]), parseFloat(st[2]) );
        });
        console.timeEnd( 'vertices' );

        console.time( 'faces' );
        var edges = [];
        var fs = str.match(/f\s.*/g);
        fs.map( function( f,i ){
            var st = f.replace( /f\s+/, '').split( /\s/ );
            edges.push( parseFloat(st[0])-1,
                        parseFloat(st[1])-1,
                        parseFloat(st[2])-1 );
        });
        console.timeEnd( 'faces' );

        cb( vertices, edges );

    };

    return exports;
}({});