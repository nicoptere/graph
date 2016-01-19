var Edge = function(){

    function Edge( v0, v1, data ){

        this.v0 = v0;
        this.v1 = v1;
        if( v0.neighbours.indexOf( v1 ) == - 1 )v0.neighbours.push( v1 );
        if( v1.neighbours.indexOf( v0 ) == - 1 )v1.neighbours.push( v0 );
    }

    function clone( deep ){

        deep = ( deep == null ) ? false : Boolean( deep );

        var e;
        if( deep ){

            e = new Edge(this.v0.clone(), this.v1.clone(), this.data );

        }else{

            e = new Edge( this.v0, this.v1, this.data );
        }
        e.id = this.id;
        return e;
    }

    function equals( other ){

        return (this.v0 === other.v0 && this.v1 === other.v1) || (this.v0 === other.v1 && this.v1 === other.v0);
    }

    function flip(){

        return new Edge(this.v1, this.v0);
    }

    function other( other ){

        return ( this.v0 === other ) ? this.v1 : this.v0;
    }

    var _p = Edge.prototype;
    _p.constructor = Edge;
    _p.clone = clone;
    _p.equals = equals;
    _p.flip = flip;
    _p.other = other;
    return Edge;

}();