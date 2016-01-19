var Graph = function(){

    var undef;
    function Graph( vertices, edges ){

        this.vertices = vertices || [];
        this.edges = edges || [];

    }

    function addVertex( vertex ){

        this.vertices.push( vertex );
    }

    function removeVertex( vertex, deleteEdges ){

        if( vertex == null )return;

        deleteEdges = ( deleteEdges == null ) ? false : Boolean( deleteEdges );

        var id = this.vertices.indexOf( vertex );
        if( id != -1 )
        {
            if( deleteEdges ){
                var scope = this;
                this.getEdgesByVertex( vertex ).forEach( function( e ){
                    scope.removeEdge( e, false );
                });
            }
            this.vertices.splice( id, 1 );
        }
    }

    function removeVertexAt( id, deleteEdges ){
        return this.removeVertex( this.vertices[id], deleteEdges );
    }



    function addEdge( edge ){

        this.edges.push( edge );
    }

    function removeEdge( edge, deleteVertices ){

        if( edge == null )return;
        deleteVertices = ( deleteVertices == null ) ? false : Boolean( deleteVertices );

        var id = this.edges.indexOf( edge );
        if( id != -1 )
        {
            if( deleteVertices ){
                this.removeVertex( edge.v0, false );
                this.removeVertex( edge.v1, false );
            }
            this.edges.splice( id, 1 );
        }
    }
    function getEdgesByVertex( vertex ){

        var edges = [];
        this.edges.forEach( function( e )
        {
            if( e.v0 == vertex || e.v1 == vertex ) edges.push( e );
        });
        return edges;
    }
    function removeEdgeAt( id, deleteVertices ){
        return this.removeEdge( this.edges[id], deleteVertices );
    }

    //finds all vertices connected to one vertex only
    function getEnds(){

        var ends = [];
        this.vertices.forEach( function( v ){ v.connexions = 0; });
        this.edges.forEach( function( e )
        {
            e.v0.connexions = e.v0.connexions++;
            e.v1.connexions = e.v1.connexions++;
        });

        this.vertices.forEach( function( v ){
            if( v.connexions <= 1 )ends.push( v );
        } );
        this.ends =ends;
        return ends;
    }

    function flush(){

        this.vertices.forEach( function( v ){
            v.neighbours = [];
            v.edges = [];
        } );

        this.edges.forEach( function( e ){
            if( e.v0.neighbours.indexOf(e.v1) == -1 ){
                e.v0.neighbours.push( e.v1 );
            }
            if( e.v1.neighbours.indexOf(e.v0) == -1 ) e.v1.neighbours.push( e.v0 );
        } );
    }

    function removeDuplicateEdges()
    {
        var tmp = [];
        var scope = this;
        this.edges.forEach( function( e )
        {
            var exists = false;
            tmp.forEach( function( o ){
                if( exists ) return;
                if( o.equals( e ) )exists = true;
            });
            if( !exists )tmp.push( e );
        } );
        this.edges = tmp;
    }

    /*
    turns the graph into a dictionary so you can access vertices directly like:  G[ key ] -> vertex
    @param key the property of the vertex that will be used as a dictionary key
     */
    function buildDictionary( key )
    {
        var scope = this;
        this.vertices.forEach( function( v )
        {
            scope[ v[ key ] ] = v;
        });
        return this;
    }

    function collapse( deep ){

        var scope = this;
        var vertices = [];
        var edges = [];
        var v0, v1;
        this.edges.forEach( function( e ){

            v0 = scope.vertices[ scope.vertices.indexOf(e.v0) ];
            v1 = scope.vertices[ scope.vertices.indexOf(e.v1) ];
            if( Boolean( deep ) ){
                v0 = v0.clone();
                v1 = v1.clone();
            }
            vertices.push( v0, v1 );
            edges.push( new Edge( v0, v1 ) );

        });
        this.vertices = vertices;
        this.edges = edges;
    }

    function clone(){

        var scope = this;
        //stores connection informations
        var bindings = [];
        this.edges.forEach( function( e ){
            bindings.push( [
                scope.vertices.indexOf(e.v0),
                scope.vertices.indexOf(e.v1)
            ] )
        });
        //clone vertices
        var vertices = [];
        this.vertices.forEach( function( v ){
            vertices.push( v.clone() );
        });
        var edges = [];
        bindings.forEach( function( bind ){
            edges.push( new Edge( vertices[ bind[0] ], vertices[ bind[1] ]));
        });
        return new Graph( vertices, edges );
    }

    var _p = Graph.prototype;
    _p.constructor = Graph;

    _p.addVertex = addVertex;
    _p.removeVertex = removeVertex;
    _p.removeVertexAt = removeVertexAt;

    _p.addEdge = addEdge;
    _p.removeEdge = removeEdge;
    _p.getEdgesByVertex = getEdgesByVertex;
    _p.removeEdgeAt = removeEdgeAt;

    _p.getEnds = getEnds;
    _p.flush = flush;
    _p.removeDuplicateEdges = removeDuplicateEdges;
    _p.buildDictionary = buildDictionary;
    _p.collapse = collapse;
    _p.clone = clone;

    return Graph;

}();
