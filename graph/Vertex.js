
var Vertex = function(){

    function Vertex( params ){

        for( var key in params) {
            this[key] = params[key];
        }
        this.neighbours = [];
    }

    function clone(){

        var out = new Vertex();
        for( var key in this){

            out[ key ] = this[key];
        }
        return out;
    }

    var _p = Vertex.prototype;
    _p.constructor = Vertex;
    _p.clone = clone;
    return Vertex;

}();