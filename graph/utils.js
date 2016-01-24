var utils = function(exports){

    /*
    compute edges length
     */
    exports.length = function( edge ){
        return Math.abs( edge.v0 - edge.v1);
    };

    exports.length2d = function( edge ){
        var dx = edge.v0.x - edge.v1.x;
        var dy = edge.v0.y - edge.v1.y;
        return Math.sqrt( dx*dx + dy*dy );
    };

    exports.length3d = function( edge ){
        var dx = edge.v0.x - edge.v1.x;
        var dy = edge.v0.y - edge.v1.y;
        var dz = edge.v0.z - edge.v1.z;
        return Math.sqrt( dx*dx + dy*dy + dz*dz  );
    };

    /*
    finds the Kruskal minimum spanning tree of a graph
     */
    exports.mst = function( exports ){

        var parent = [];
        var rank = [];
        var tree = [];
        var sortedEdges = [];

        //function _metric( edge ){
        //
        //    var dx = edge.v0.x - edge.v1.x;
        //    var dy = edge.v0.y - edge.v1.y;
        //    return Math.sqrt( dx*dx + dy*dy );
        //}

        function _sortMethod( edge0, edge1 ){
            return edge0.weight - edge1.weight;
        }

        function find( vertex ){
            if( parent[ vertex.id ] == vertex ){
                return parent[ vertex.id ];
            }else{
                return find( parent[ vertex.id ] );
            }
        }

        function union( root0, root1 ){

            var id0 = root0.id;
            var id1 = root1.id;

            if( rank[ id0 ] > rank[ id1 ] ){
                parent[ id1 ] = root0;
            }
            else if( rank[ id0 ] < rank[ id1 ] ){
                parent[ id0 ] = root1;
            }else{
                parent[ id0 ] = root1;
                rank[ id1 ]++;
            }
        }

        /*
         * computes and returns a Minimum Spanning Tree using the Kruskal Algorithm
         * @param graph object to compute the MST of
         * @param metric a function( edge ) to compute the weight of the edge
         * @param sortMethod a sort function( edge0, edge1 ) to sort edges by increasing weight
         * @constructor
         */
        function compute( graph, metric, sortMethod ){

            metric = metric || utils.length2d;
            sortMethod = sortMethod || _sortMethod;

            parent = [];
            rank = [];
            tree = [];

            graph.vertices.forEach( function( vertex, i ){
                vertex.id = i;
                parent[ vertex.id ] = vertex;
                rank[ vertex.id ] = 0;
            } );

            sortedEdges = graph.edges.concat();
            sortedEdges.forEach( function( e,i ){
                e.id = i;
                e.weight = metric( e );
            });
            sortedEdges.sort( sortMethod );

            for ( var i = 0; i < sortedEdges.length; i++){

                var edge = sortedEdges[ i ];
                var root1 = find( edge.v0 );
                var root2 = find( edge.v1 );
                if( root1 != root2 ){
                    tree.push( edge );
                    union( root1, root2 );
                }
                if( tree.length == graph.vertices.length - 1 ){
                    return tree;
                }
            }
            return tree;
        }
        exports.compute = compute;
        return exports;

    }({});

    //http://www.briangrinstead.com/blog/astar-search-algorithm-in-javascript
    exports.astar = function( exports ){


        exports.dist = function(a,b){
            var dx =( a.x - b.x );
            var dy =( a.y - b.y );
            return Math.sqrt( dx*dx + dy*dy );
        };

        exports.search = function(graph, start, end, metric ) {

            exports.metric = metric || exports.dist;

            graph.vertices.forEach( function( v ){
                v.f = 0;
                v.g = 0;
                v.h = 0;
                v.parent = null;
            });

            var visited = [];
            var unvisited   = [];
            unvisited.push( start );

            while(unvisited.length > 0) {

                // Grab the lowest f(x) to process next
                var lowInd = 0;
                for( var i = 0; i<unvisited.length; i++) {
                    if( unvisited[i].f < unvisited[lowInd].f ){
                        lowInd = i;
                    }
                }

                var currentNode = unvisited[lowInd];

                // End case -- result has been found, return the traced path
                if(currentNode == end ) {
                    var curr = currentNode;
                    var ret = [];
                    while(curr.parent) {
                        ret.push(curr);
                        curr = curr.parent;
                    }
                    ret.reverse();
                    ret.unshift( start );
                    return ret;
                }

                // Normal case -- move currentNode from open to closed, process each of its neighbors
                unvisited.splice(currentNode,1);
                visited.push(currentNode);

                var neighbors = currentNode.neighbours;
                for( i = 0; i < neighbors.length; i++ ) {

                    var neighbor = neighbors[i];
                    if( visited.indexOf( neighbor ) != -1 ) {
                        // not a valid node to process, skip to next neighbor
                        continue;
                    }

                    // g score is the shortest distance from start to current node, we need to check if
                    //	 the path we have arrived at this neighbor is the shortest one we have seen yet
                    var gScore = currentNode.g + exports.metric( currentNode, neighbor ); // 1 is the distance from a node to it's neighbor
                    var gScoreIsBest = false;

                    if( unvisited.indexOf(neighbor) == -1 ) {
                        // This the the first time we have arrived at this node, it must be the best
                        // Also, we need to take the h (heuristic) score since we haven't done so yet
                        neighbor.h = exports.metric( neighbor, end );
                        unvisited.push(neighbor);
                        gScoreIsBest = true;
                    }
                    else if(gScore < neighbor.g) {
                        // We have already seen the node, but last time it had a worse g (distance from start)
                        gScoreIsBest = true;
                    }

                    if(gScoreIsBest) {
                        // Found an optimal (so far) path to this node.	 Store info on how we got here and
                        //	just how good it really is...
                        neighbor.parent = currentNode;
                        neighbor.g = gScore;
                        neighbor.f = neighbor.g + neighbor.h;
                    }
                }
            }
            return [];
        };
        return exports;
    }({});


    /**
     computes the Dijkstra shortest distance on a graph
     */
    exports.dijkstra = function( exports ) {

        exports.edgeLength2d = function( a, b ){
            var dx = ( a.x - b.x );
            var dy = ( a.y - b.y );
            return Math.sqrt( dx*dx + dy*dy );
        };
        exports.edgeLength3d = function( a, b ){
            var dx = ( a.x - b.x );
            var dy = ( a.y - b.y );
            return Math.sqrt( dx*dx + dy*dy );
        };

        //buildis adjacency list
        function init( graph, root, metric) {

            metric = metric || exports.edgeLength2d;

            var dist = new Float32Array(graph.vertices.length);
            var adjacent = [];
            var cost = [];
            var prev = [];
            var queue = [];

            //inits a list
            graph.vertices.forEach(function(v, i){

                v.prev = null;

                //initializes the distance to root at 0 ( ensures that root is the closest node to itself )
                dist[ i ] = v == root ? 0 : Number.POSITIVE_INFINITY;

                //stores the adjacency and the edges costs
                var adj = new Int16Array(v.neighbours.length);
                var cos = new Float32Array(v.neighbours.length);

                v.neighbours.forEach( function(n, j)
                {
                    adj[ j ] = graph.vertices.indexOf( n );
                    cos[ j ] = metric( v, n );
                } );
                adjacent.push( adj );
                cost.push( cos );

                queue.push( i );

            });

            var current, i, id, vertex, alt, min;
            while( queue.length > 0 ){

                //select the closest vertex
                min = Number.POSITIVE_INFINITY;
                for( i = 0; i < queue.length; i++ ){
                    id = queue[i];
                    if ( dist[ id ] < min) {
                        min = dist[ id ];
                        current = id;
                    }
                }

                //removes current node from the queue
                queue.splice( queue.indexOf( current ), 1 );

                //for all neighbours of current, update distance to target node
                for( i = 0; i < adjacent[ current ].length; i++ ){

                    vertex = adjacent[current][i];

                    alt = dist[ current ] + cost[ current ][ i ];

                    if (alt < dist[ vertex ] ) {

                        dist[ vertex ] = alt;
                        prev[ vertex ] = current;
                    }
                }
            }
            prev.forEach(function(id, i){
                graph.vertices[ i ].prev = graph.vertices[ id ];
            });
        }

        //computes and returns shortest path from the root to the target vertices
        function getShortestPath( target ){
            var nodes = [];
            var tg = target;

            //target doesn't belong to the graph
            if( tg == null )return nodes;
            //otherwise list all nodes from the target to the root node
            while( tg.prev != null ){
                nodes.unshift( tg );
                tg = tg.prev;
            }
            nodes.unshift( tg );
            return nodes;
        }

        exports.init = init;
        exports.getShortestPath = getShortestPath;
        return exports;

    }({});

    /**
     * do not use, left it for "learning purpose"
     */
    exports.flawedDijkstra = function( exports ){

        //buildis adjacency list
        function init( graph, root, metric ){

            metric = metric || utils.length2d;
            graph.edges.forEach(function (edge) {

                var d = metric( edge );

                //this is the flaw...
                if( edge.v0.cost == null )edge.v0.cost = 0;
                if( edge.v1.cost == null )edge.v1.cost = 0;
                edge.v0.cost = Math.max( edge.v0.cost, d );
                edge.v1.cost = Math.max( edge.v1.cost, d );

                edge.v0.dist = Number.POSITIVE_INFINITY;
                edge.v1.dist = Number.POSITIVE_INFINITY;
                edge.v0.previous = null;
                edge.v1.previous = null;
            });

            //initializes the distance to root at 0 ( ensures that root is the closest node to itself )
            root.dist = 0;

            var current, queue = graph.vertices.concat();
            while (queue.length > 0){

                //select the closest vertex
                var min = Number.POSITIVE_INFINITY;
                queue.forEach(function (vertex){

                    if( vertex.dist < min){

                        min = vertex.dist;
                        current = vertex;
                    }
                });

                //removes current node from the queue
                queue.splice(queue.indexOf(current), 1);

                if (current.dist == Number.POSITIVE_INFINITY) break;

                //for all neighbours of current, update distance to target node
                current.neighbours.forEach(function (vertex)
                {
                    var alt = current.dist + vertex.cost;
                    if (alt < vertex.dist){

                        vertex.dist = alt;
                        vertex.previous = current;
                    }
                });
            }
        }

        //computes and returns shortest path from the root to the target vertices
        function getShortestPath( target ){

            var nodes = [];
            var tg = target;

            //target doesn't belong to the graph
            if( tg == null )return nodes;

            //otherwise list all nodes from the target to the root node
            while( tg.previous != null ){

                nodes.unshift( tg );
                tg = tg.previous;
            }
            nodes.unshift( tg );
            return nodes;
        }

        exports.init = init;
        exports.getShortestPath = getShortestPath;
        return exports;

    }({});

    /**
     * builds a cubic graph that contain a Hamiltonian cycle after it's LCF notation
     * https://en.wikipedia.org/wiki/LCF_notation
     * http://mathworld.wolfram.com/LCFNotation.html
     */
    exports.LCF = function(exports){

        exports.build = function( code  ) {
            //parse the expression

            //gets the operations
            var operations = code.match(/(\[.*\])/);
            var cycle = operations[1].substr( 1, operations[1].length - 1).split( ',').map( function(v){ return parseInt( v );});

            //gets the vertices count
            var count;
            var c = code.match( /]([0-9]+)/ );
            if( c == null ){
                count = cycle.length;
            }else {
                count = parseInt( c[1] ) * cycle.length;
            }

            //builds the vertices
            var vertices = [];
            for( var i = 0; i < count; i++ ){
                var v = new Vertex( { x:Math.random() - .5,y:Math.random() - .5,z:Math.random() - .5 });
                vertices.push( v );
            }

            //build the inital chain
            var edges = [];
            for( i = 0; i < count; i++ ){
                var e = new Edge( vertices[i], vertices[(i+1)%vertices.length] );
                edges.push( e );
            }

            //applies the rule
            var ruleId = 0;
            for( i = 0; i < count; i++ ){

                var id = ( i + cycle[ ruleId % cycle.length ] ) % vertices.length;
                if( id < 0 )id += vertices.length;

                edges.push( new Edge( vertices[i], vertices[id] ) );
                ruleId++;
            }

            return new Graph( vertices, edges );

        };
        return exports;
    }({});


    /**
     * performs a Delaunay triangulation on a set of 2D vertices [x,y]
     *
     *  https://github.com/ironwallaby/delaunay/blob/master/delaunay.js
     */
    exports.delaunay = function( exports ){

        var EPSILON = 1.0 / 1048576.0;
        function supertriangle(vertices){
            var xmin = Number.POSITIVE_INFINITY, ymin = Number.POSITIVE_INFINITY, xmax = Number.NEGATIVE_INFINITY, ymax = Number.NEGATIVE_INFINITY, i, dx, dy, dmax, xmid, ymid;

            for(i = vertices.length; i--; ) {
                if(vertices[i].x < xmin) xmin = vertices[i].x;
                if(vertices[i].x > xmax) xmax = vertices[i].x;
                if(vertices[i].y < ymin) ymin = vertices[i].y;
                if(vertices[i].y > ymax) ymax = vertices[i].y;
            }
            dx = xmax - xmin;
            dy = ymax - ymin;
            dmax = Math.max(dx, dy);
            xmid = xmin + dx * 0.5;
            ymid = ymin + dy * 0.5;
            return [
                {x:xmid - 20 * dmax, y:ymid -      dmax},
                {x:xmid            , y:ymid + 20 * dmax},
                {x:xmid + 20 * dmax, y:ymid -      dmax}
            ];
        }
        function circumcircle(vertices, i, j, k){

            var x1 = vertices[i].x,
                y1 = vertices[i].y,
                x2 = vertices[j].x,
                y2 = vertices[j].y,
                x3 = vertices[k].x,
                y3 = vertices[k].y,
                fabsy1y2 = Math.abs(y1 - y2),
                fabsy2y3 = Math.abs(y2 - y3),
                xc, yc, m1, m2, mx1, mx2, my1, my2, dx, dy;
            if(fabsy1y2 < EPSILON && fabsy2y3 < EPSILON)throw new Error("Eek! Coincident points!");
            if(fabsy1y2 < EPSILON) {
                m2  = -((x3 - x2) / (y3 - y2));
                mx2 = (x2 + x3) / 2.0;
                my2 = (y2 + y3) / 2.0;
                xc  = (x2 + x1) / 2.0;
                yc  = m2 * (xc - mx2) + my2;
            }else if(fabsy2y3 < EPSILON) {
                m1  = -((x2 - x1) / (y2 - y1));
                mx1 = (x1 + x2) / 2.0;
                my1 = (y1 + y2) / 2.0;
                xc  = (x3 + x2) / 2.0;
                yc  = m1 * (xc - mx1) + my1;
            }else {
                m1  = -((x2 - x1) / (y2 - y1));
                m2  = -((x3 - x2) / (y3 - y2));
                mx1 = (x1 + x2) / 2.0;
                mx2 = (x2 + x3) / 2.0;
                my1 = (y1 + y2) / 2.0;
                my2 = (y2 + y3) / 2.0;
                xc  = (m1 * mx1 - m2 * mx2 + my2 - my1) / (m1 - m2);
                yc  = (fabsy1y2 > fabsy2y3) ?
                m1 * (xc - mx1) + my1 :
                m2 * (xc - mx2) + my2;
            }
            dx = x2 - xc;
            dy = y2 - yc;
            return {i: i, j: j, k: k, x: xc, y: yc, r: dx * dx + dy * dy};
        }
        function dedup(edges){
            var i, j, a, b, m, n;
            for(j = edges.length; j; ) {
                b = edges[--j];
                a = edges[--j];
                for(i = j; i; ) {
                    n = edges[--i];
                    m = edges[--i];

                    if((a === m && b === n) || (a === n && b === m)) {
                        edges.splice(j, 2);
                        edges.splice(i, 2);
                        break;
                    }
                }
            }
        }
        function compute(vertices){

            var n = vertices.length,i, j, indices, st, open, closed, edges, dx, dy, a, b, c;
            if(n < 3)return [0,1,2];
            vertices = vertices.slice(0);
            indices = new Array(n);
            for(i = n; i--; )indices[i] = i;
            indices.sort(function(i, j) { return vertices[j].x - vertices[i].x; });
            st = supertriangle(vertices);
            vertices.push(st[0], st[1], st[2]);
            open   = [circumcircle(vertices, n + 0, n + 1, n + 2)];
            closed = [];
            edges  = [];
            for(i = indices.length; i--; edges.length = 0) {
                c = indices[i];
                for(j = open.length; j--; ){
                    dx = vertices[c].x - open[j].x;
                    if(dx > 0.0 && dx * dx > open[j].r) {
                        closed.push(open[j]);
                        open.splice(j, 1);
                        continue;
                    }
                    dy = vertices[c].y - open[j].y;
                    if(dx * dx + dy * dy - open[j].r > EPSILON)continue;
                    edges.push(
                        open[j].i, open[j].j,
                        open[j].j, open[j].k,
                        open[j].k, open[j].i
                    );
                    open.splice(j, 1);
                }
                dedup(edges);
                for(j = edges.length; j; ) {
                    b = edges[--j];
                    a = edges[--j];
                    open.push(circumcircle(vertices, a, b, c));
                }
            }
            for(i = open.length; i--; ){
                closed.push(open[i]);
            }
            open.length = 0;

            for(i = closed.length; i--; ){
                if(closed[i].i < n && closed[i].j < n && closed[i].k < n)
                {
                    open.push(closed[i].i, closed[i].j, closed[i].k);
                }
            }
            return open;
        }
        function contains(tri, p){
            if((p.x< tri[0].x && p.x< tri[1].x && p.x< tri[2].x) ||
                (p.x> tri[0].x && p.x> tri[1].x && p.x> tri[2].x) ||
                (p.y< tri[0].y && p.y< tri[1].y && p.y< tri[2].y) ||
                (p.y> tri[0].y && p.y> tri[1].y && p.y> tri[2].y))
                return null;
            var a = tri[1].x - tri[0].x, b = tri[2].x - tri[0].x, c = tri[1].y - tri[0].y, d = tri[2].y - tri[0].y, i = a * d - b * c;
            if(i === 0.0) return null;
            var u = (d * (p.x- tri[0].x) - b * (p.y- tri[0].y)) / i,
                v = (a * (p.y- tri[0].y) - c * (p.x- tri[0].x)) / i;
            if(u < 0.0 || v < 0.0 || (u + v) > 1.0) return null;
            return [u, v];
        }
        exports.compute = compute;
        return exports;
    }({});

    /**
     * performs a delaunay triangulation of a vertices set
     * @param graph
     * @returns {*}
     */
    exports.triangulate = function( graph, deleteEdges ){

        if( graph.vertices.length < 2 )throw new Error( "utils.triangulate: not enough vertices to build graph." );

        if( Boolean( deleteEdges ) )graph.edges = [];
        if( graph.vertices.length == 2 ){
            graph.addEdge( new Edge( graph.vertices[0], graph.vertices[1] ) );
            return;
        }
        var indices = utils.delaunay.compute( graph.vertices );
        for( i = 0; i < indices.length; i+=3 ){

            var i0 = indices[ i ];
            var i1 = indices[ i+1 ];
            var i2 = indices[ i+2 ];
            graph.addEdge( new Edge( graph.vertices[i0], graph.vertices[i1] ) );
            graph.addEdge( new Edge( graph.vertices[i1], graph.vertices[i2] ) );
            graph.addEdge( new Edge( graph.vertices[i2], graph.vertices[i0] ) );
        }
        return graph;
    };

    /**
     * performs a triangulation and traingulates the triangles' centers
     * @param graph
     * @returns {*}
     */
    exports.triangulateCenters = function( graph ){

        if( graph.vertices.length < 2 )throw new Error( "utils.triangulate: not enough vertices to build graph." );
        if( graph.vertices.length == 2 ){
            g.addEdge( new Edge( graph.vertices[0], graph.vertices[1] ) );
            return;
        }
        var vertices = [];
        var indices = utils.delaunay.compute( graph.vertices );
        for( i = 0; i < indices.length; i+=3 ){

            var i0 = indices[ i ];
            var i1 = indices[ i+1 ];
            var i2 = indices[ i+2 ];
            var x = ( graph.vertices[i0].x + graph.vertices[i1].x + graph.vertices[i2].x ) / 3;
            var y = ( graph.vertices[i0].y + graph.vertices[i1].y + graph.vertices[i2].y ) / 3;
            vertices.push( new Vertex( {x:x, y:y } ) );
        }
        var g = new Graph(vertices);
        return utils.triangulate( g );

    };

    /**
     * removes a given amount of vertices
     * @param graph the graph to process
     * @param percentage of vertices to remove [0-1]
     * @returns {*}
     */
    exports.decimateVertices = function( graph, ratio ){

        for( var i = graph.vertices.length - 1; i > 0 ; i-- ){
            if( Math.random()<ratio ){
                graph.removeVertexAt(  i, true );
            }
        }
        return graph;
    };
    /**
     * removes a given amount of vertices
     * @param graph the graph to process
     * @param percentage of edges to remove [0-1]
     * @returns {*}
     */
    exports.decimateEdges = function( graph, ratio ){

        for( var i = graph.edges.length - 1; i > 0 ; i-- ){
            if( PRNG.random()<ratio ){
                graph.removeEdgeAt(  i, false );
            }
        }
        return graph;
    };

    function lerp ( t, a, b ){ return a + t * ( b - a ); }
    function norm( t, a, b ){return ( t - a ) / ( b - a );}
    function map(t, a0, b0, a1, b1) { return a1 + ( t - a0 ) / ( b0 - a0 ) * ( b1 - a1 ); }

    /**
     * rescales a graph from one coordinate system to another
     * @param g the original grpah
     * @param from the original bounds described as [ x0, y0, x1, y1 ]
     * @param to the destination bounds described as [ x0, y0, x1, y1 ]
     * @returns {*}
     */
    exports.remap = function( g, from, to ){

        g.vertices.forEach( function( v ){

            v.x = map(v.x, from[ 0 ], from[ 2 ], to[ 0 ], to[ 2 ] );
            v.y = map(v.y, from[ 1 ], from[ 3 ], to[ 1 ], to[ 3 ] );
        });
        return g;
    };

    /*
     renders a 2D graph on a canvas
     */
    exports.render = function( graph, ctx, pointSize, lineSize ){

        //ctx.lineJoin = "round";
        exports.renderEdges(graph, ctx, lineSize );

        //ctx.lineJoin = "mitter";
        exports.renderVertices(graph, ctx, pointSize );
    };
    exports.renderEdges = function( graph, ctx, lineSize ){

        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.lineWidth = lineSize || .5;
        ctx.beginPath();
        graph.edges.forEach( function( e ){
            ctx.moveTo(e.v0.x, e.v0.y );
            ctx.lineTo(e.v1.x, e.v1.y );
        });
        ctx.stroke();
    };
    exports.renderVertices = function( graph, ctx, pointSize ){
        ctx.lineCap = "round";
        ctx.lineWidth = pointSize || 3;
        ctx.beginPath();
        if( pointSize != 0 ){
            graph.vertices.forEach(function(v)
            {
                ctx.moveTo( v.x, v.y );
                ctx.lineTo(v.x +.5, v.y);
            })
        }
        ctx.stroke();
    };

    exports.renderLabel = function( graph, ctx ){

        ctx.font = "10px Verdana";
        ctx.fillStyle = "#000";
        graph.vertices.forEach( function( v,i ){
            ctx.fillText( i, v.x, v.y - 10 );
        });
    };
    return exports;
}({});
