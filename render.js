(function(global){
	
	var Vector4 = function(x,y,z,w=1){
		this.X = x;
		this.Y = y;
		this.Z = z;
		this.W = w;
	}
	
	Vector4.prototype.length = function(){
		return Math.sqrt(this.X*this.X+this.Y*this.Y+this.Z*this.Z+this.W*this.W);
	}
	
	Vector4.prototype.norm = function(){
		let len = this.length();
		this.X /= len;
		this.Y /= len;
		this.Z /= len;
	}
	
	Vector4.prototype.reduce = function(right){
		return new Vector4(this.X-right.X,this.Y-right.Y,this.Z-right.Z,this.W-right.W);
	}
	
	Vector4.prototype.cross = function(right){
		return new Vector4(this.Y*right.Z - this.Z*right.Y,
						   this.Z*right.X - this.X*right.Z,
						   this.X*right.Y - this.Y*right.X, 
						   0);
	}
	
	Vector4.prototype.dot = function(right){
		return this.X*right.X + this.Y*right.Y + this.Z*right.Z + this.W*right.W;
	}
	
	var Matrix4x4 = function(
			A11, A12, A13, A14,
            A21, A22, A23, A24,
            A31, A32, A33, A34,
            A41, A42, A43, A44){
	    this.A11 = A11, this.A12 = A12, this.A13 = A13, this.A14 = A14,
        this.A21 = A21, this.A22 = A22, this.A23 = A23, this.A24 = A24,
        this.A31 = A31, this.A32 = A32, this.A33 = A33, this.A34 = A34,
        this.A41 = A41, this.A42 = A42, this.A43 = A43, this.A44 = A44;		
	}
	
	Matrix4x4.prototype.multiplyMat = function(right){
		let r11 = (this.A11 * right.A11) + (this.A12 * right.A21) + (this.A13 * right.A31) + (this.A14 * right.A41);
        let r12 = (this.A11 * right.A12) + (this.A12 * right.A22) + (this.A13 * right.A32) + (this.A14 * right.A42);
        let r13 = (this.A11 * right.A13) + (this.A12 * right.A23) + (this.A13 * right.A33) + (this.A14 * right.A43);
        let r14 = (this.A11 * right.A14) + (this.A12 * right.A24) + (this.A13 * right.A34) + (this.A14 * right.A44);

        let r21 = (this.A21 * right.A11) + (this.A22 * right.A21) + (this.A23 * right.A31) + (this.A24 * right.A41);
        let r22 = (this.A21 * right.A12) + (this.A22 * right.A22) + (this.A23 * right.A32) + (this.A24 * right.A42);
        let r23 = (this.A21 * right.A13) + (this.A22 * right.A23) + (this.A23 * right.A33) + (this.A24 * right.A43);
        let r24 = (this.A21 * right.A14) + (this.A22 * right.A24) + (this.A23 * right.A34) + (this.A24 * right.A44);

        let r31 = (this.A31 * right.A11) + (this.A32 * right.A21) + (this.A33 * right.A31) + (this.A34 * right.A41);
        let r32 = (this.A31 * right.A12) + (this.A32 * right.A22) + (this.A33 * right.A32) + (this.A34 * right.A42);
        let r33 = (this.A31 * right.A13) + (this.A32 * right.A23) + (this.A33 * right.A33) + (this.A34 * right.A43);
        let r34 = (this.A31 * right.A14) + (this.A32 * right.A24) + (this.A33 * right.A34) + (this.A34 * right.A44);

        let r41 = (this.A41 * right.A11) + (this.A42 * right.A21) + (this.A43 * right.A31) + (this.A44 * right.A41);
        let r42 = (this.A41 * right.A12) + (this.A42 * right.A22) + (this.A43 * right.A32) + (this.A44 * right.A42);
        let r43 = (this.A41 * right.A13) + (this.A42 * right.A23) + (this.A43 * right.A33) + (this.A44 * right.A43);
        let r44 = (this.A41 * right.A14) + (this.A42 * right.A24) + (this.A43 * right.A34) + (this.A44 * right.A44);
		return new Matrix4x4(r11, r12, r13, r14, r21, r22, r23, r24, r31, r32, r33, r34, r41, r42, r43, r44);
	}
	
	Matrix4x4.prototype.multiplyVec4 = function(right){
		let x = (this.A11 * right.X) + (this.A12 * right.Y) + (this.A13 * right.Z) + (this.A14 * right.W);
        let y = (this.A21 * right.X) + (this.A22 * right.Y) + (this.A23 * right.Z) + (this.A24 * right.W);
        let z = (this.A31 * right.X) + (this.A32 * right.Y) + (this.A33 * right.Z) + (this.A34 * right.W);
        let w = (this.A41 * right.X) + (this.A42 * right.Y) + (this.A43 * right.Z) + (this.A44 * right.W);
		return new Vector4(x,y,z,w);
	}
	
	function identify(){
		return new Matrix4x4(1,0,0,0,
							 0,1,0,0,
							 0,0,1,0,
							 0,0,0,1);
	}

	
	function lookAt(position_v4, target_v4, up_v4){
		let z = position_v4.reduce(target_v4);
		z.norm();
		let x = up_v4.cross(z);
		x.norm();
		let y = z.cross(x);
		return new Matrix4x4(x.X,x.Y,x.Z, -x.dot(position_v4),
							 y.X,y.Y,y.Z, -y.dot(position_v4),
							 z.X,z.Y,z.Z, -z.dot(position_v4),
							 0,0,0,1);
	}
	
	function perspectProjecton(aspect, fov, near, far){
		let rad = fov * Math.PI / 180;
		let top = near * Math.tan(rad/2), bottom = -top, right = aspect*top, left=-right;
		return new Matrix4x4(2*near/(right-left),0,(right+left)/(right-left),0,
							 0,2*near/(top-bottom),(top+bottom)/(top-bottom),0,
							 0,0,-(near+far)/(far-near),-(2*near*far)/(far-near),
							 0,0,-1,0);
	}
	
	function rotationX(angle){
		let rad = angle* Math.PI / 180;
		let cos = Math.cos(rad), sin = Math.sin(rad);
		return new Matrix4x4(1,0,0,0,
							 0,cos,-sin,0,
							 0,sin,cos,0,
							 0,0,0,1);
	}
	
	function rotationY(angle){
		let rad = angle* Math.PI / 180;
		let cos = Math.cos(rad), sin = Math.sin(rad);
		return new Matrix4x4(cos,0,sin,0,
							 0,1,0,0,
							 -sin,0,cos,0,
							 0,0,0,1);
	}
	
	function rotationZ(angle){
		let rad = angle* Math.PI / 180;
		let cos = Math.cos(rad), sin = Math.sin(rad);
		return new Matrix4x4(cos,-sin,0,0,
							 sin,cos,0,0,
							 0,0,1,0,
							 0,0,0,1);
	}
	
	const GL_BUFFER_TYPE = {
		UNSIGNED_BYTE:  Uint8Array,
		UNSIGNED_SHORT: Uint16Array,
		UNSIGNED_INT:   Uint32Array,
		FLOAT:          Float32Array
	};
	Object.freeze(GL_BUFFER_TYPE);
	
	const Mode = {
		POINTS:    0,
		LINES:     1,
		TRIANGLES: 2,
	};
	Object.freeze(Mode);
	
	
	var VertexAttribPointer = function(type_gl_t, size, stride, offset){
		this.type = type_gl_t;
		this.size = size;
		this.stride = stride;
		this.offset = offset;
	}
	
	VertexAttribPointer.prototype.vertexAttribPointerToView = function(index, vab){
		let offset = index * this.stride + this.offset;
		return new this.type(vab, offset, this.size);	
	}
	
	
	var FrameBuffer = function(width, height, color_vec4, depth_f){
		//RGBA buffer
		this.colorBuffer = new Uint8Array(width*height*4);
		this.depthBuffer = new Float32Array(width*height)
		this.clearColor = color_vec4;
		this.clearDepth = depth_f;
		this.width = width;
		this.height = height;
	}
	
	FrameBuffer.prototype.setColor = function(x, y, color_vec4){
		let index = Math.floor(x + y * this.width);
		this.colorBuffer[index*4]     = color_vec4.X;
		this.colorBuffer[index*4 + 1] = color_vec4.Y;
		this.colorBuffer[index*4 + 2] = color_vec4.Z;
		this.colorBuffer[index*4 + 3] = color_vec4.W;
	}
	
	FrameBuffer.prototype.getColor = function(x, y){
		let index = Math.floor(x + y * this.width);
		return new Vector4(this.colorBuffer[index*4],
						   this.colorBuffer[index*4 + 1],
						   this.colorBuffer[index*4 + 2],
						   this.colorBuffer[index*4 + 3]);
	}
	
	FrameBuffer.prototype.setDepth = function(x, y, depth){
		let index = Math.floor(x + y * this.width);
		this.depthBuffer[index] = depth;
	}
	
	FrameBuffer.prototype.getDepth = function(x, y){
		let index = Math.floor(x + y * this.width);
		return this.depthBuffer[index];
	}
	
	FrameBuffer.prototype.clear = function(){
		for(let x = 0; x < this.width; x++){
			for(let y = 0; y < this.height; y++){
				this.setColor(x,y,this.clearColor);
				this.setDepth(x,y,this.clearDepth);
			}
		}
	}
	
	
	var Render = function(vb, vaps, ib, fb, vs, fs){
		//must be ArrayBuffer, not TypedArray
		this.vertexArrayBuffer = vb;
		this.vertexAttribPointes = vaps;
		//int array or int TypedArray
		this.indexBuffer = ib;
		this.frameBuffer = fb;
		this.gl_positions = [];
		//program
		this.vertexShader = vs;
		this.fragmentShader = fs;
		//structure: {uniform1: a, uniform2: b}
		this.uniforms = {};
		//structure: [{attri1:[], attri2:[]}, {attri1:[], attri2:[]}]
		this.varyings = [];
	};
	
	Render.prototype.drawElements = function(mode,count,offset){
		
		//index
		let ib = this.indexBuffer.slice(offset, offset + count);
		
		//vertex shader
		for(let index of ib){
			this.varyings[index] = {};
			this.gl_positions[index] = this.vertexShader(
				//vertex attribute
				this.vertexAttribPointes.map(vap=>vap.vertexAttribPointerToView(index,this.vertexArrayBuffer,vap)), 
				//varyings 
				this.varyings[index],
				//uniforms
				this.uniforms
			);
		}
		
		// clipping
		// TODO
		
		

		// rasterization
		switch(mode){
			case 0:
				break;
			case 1:
				break;
			case 2:
				let numTriangle = ib.length / 3;
				for(let idx=0; idx<numTriangle; idx++){
					// triangle vertex varyings
					let vertex_vars_a = this.varyings[ib[idx*3]],
						vertex_vars_b = this.varyings[ib[idx*3+1]],
						vertex_vars_c = this.varyings[ib[idx*3+2]];
					// triangle vertex clip coord
					let vertex_clip_p_a = this.gl_positions[ib[idx*3]],
						vertex_clip_p_b = this.gl_positions[ib[idx*3+1]],
						vertex_clip_p_c = this.gl_positions[ib[idx*3+2]];
					// perspectiveDivide ndc coord
					let	vertex_ndc_p_a = toNDC(vertex_clip_p_a),
						vertex_ndc_p_b = toNDC(vertex_clip_p_b),
						vertex_ndc_p_c = toNDC(vertex_clip_p_c);
					// viewport transformation screen coord
					let vertex_src_p_a = viewportTransformations(vertex_ndc_p_a,this.frameBuffer),
						vertex_src_p_b = viewportTransformations(vertex_ndc_p_b,this.frameBuffer),
						vertex_src_p_c = viewportTransformations(vertex_ndc_p_c,this.frameBuffer);
					
					let area_abc = edg(vertex_src_p_a, vertex_src_p_b, vertex_src_p_c);
					
					//bound box
					let start_x = Math.min(vertex_src_p_a.X, vertex_src_p_b.X, vertex_src_p_c.X);
					let end_x = Math.max(vertex_src_p_a.X, vertex_src_p_b.X, vertex_src_p_c.X);
					let start_y = Math.min(vertex_src_p_a.Y, vertex_src_p_b.Y, vertex_src_p_c.Y);
					let end_y = Math.max(vertex_src_p_a.Y, vertex_src_p_b.Y, vertex_src_p_c.Y);
					start_x = Math.floor(Math.max(0, start_x));
					end_x = Math.min(end_x, this.frameBuffer.width);
					start_y = Math.floor(Math.max(0, start_y));
					end_y = Math.min(end_y, this.frameBuffer.height);
					
	
					let d;
					let area_abd, area_bcd, area_cad;
					let i,j,k;
					let depth;
					//draw each pixel in the bound box
					for(let y = start_y; y < end_y; y++){
						for(let x = start_x; x < end_x; x++){
							//centre of the pixel
							d = new Vector4(x+0.5,y+0.5,0);
							
							area_abd = edg(vertex_src_p_a, vertex_src_p_b, d);
							area_bcd = edg(vertex_src_p_b, vertex_src_p_c, d);
							area_cad = edg(vertex_src_p_c, vertex_src_p_a, d);
							//barycentric
							i = area_bcd / area_abc, j = area_cad / area_abc, k = area_abd / area_abc;
							
							// test pixel inside the triangle
							if(i < 0 || j < 0 || k < 0) continue;
							
							// Wclip = -Zview
							// Z reclamation,  1/Zn = i * 1/Z1 + j * 1/Z2 + k * 1/Z3;
							i /= vertex_src_p_a.W;
							j /= vertex_src_p_b.W;
							k /= vertex_src_p_c.W;

							depth = 1/(i+j+k);
							
							// depth test
							if(this.frameBuffer.getDepth(x, y) < depth) continue;
							this.frameBuffer.setDepth(x, y, depth);
							
							//barycentric reclamation
							i *= depth;
							j *= depth;
							k *= depth;
							
							let fragment_vars = {};
							//varyings interpolation
							for(const [key, value] of Object.entries(vertex_vars_a)){
							    fragment_vars[key] = [];
								let v_f = fragment_vars[key],
									v_a = vertex_vars_a[key],
									v_b = vertex_vars_b[key],
									v_c = vertex_vars_c[key];
								//varying component interpolation
								for(let idx = 0; idx < value.length; idx++){
									v_f[idx] = interpolationBarycentric(v_a[idx],v_b[idx],v_c[idx],i,j,k);
								}
							}
							// fragment shader
							let color = this.fragmentShader(fragment_vars, this.uniforms);
							this.frameBuffer.setColor(x,y,color);
							
						}
					}
				}
			
		}
	
	}
	
	function toNDC(vertex_clip_p){
		return new Vector4(vertex_clip_p.X/vertex_clip_p.W,
						   vertex_clip_p.Y/vertex_clip_p.W,
						   vertex_clip_p.Z/vertex_clip_p.W,
						   vertex_clip_p.W)
	}
	
	function viewportTransformations(vertex_ndc_p, frameBuffer){
		return new Vector4((vertex_ndc_p.X+1)* frameBuffer.width * 0.5,  //to [0, width)
						   (vertex_ndc_p.Y+1)* frameBuffer.height* 0.5,  //to [0, height)
						   (vertex_ndc_p.Z+1)*0.5,					  	 //to [0, 1)
						    vertex_ndc_p.W) 
	}
	
	function edg(a,b,c){
		return (b.X - a.X)*(c.Y - a.Y) - (b.Y - a.Y)*(c.X - a.X);
	}
	
	function interpolationBarycentric(var1,var2,var3,i,j,k){
		return var1*i + var2 *j + var3*k;
	}
	
	//export
	GL = {};
	GL.Vector4 = Vector4;
	GL.Matrix4x4 = Matrix4x4;
	GL.Matrix4x4.identify = identify;
	GL.Matrix4x4.lookAt = lookAt;
	GL.Matrix4x4.perspectProjecton = perspectProjecton;
	GL.Matrix4x4.rotationX = rotationX;
	GL.Matrix4x4.rotationY = rotationY;
	GL.Matrix4x4.rotationZ = rotationZ;
	GL.GL_BUFFER_TYPE = GL_BUFFER_TYPE;
	GL.Mode = Mode;
	GL.VertexAttribPointer = VertexAttribPointer;
	GL.FrameBuffer = FrameBuffer;
	GL.Render = Render;
	global.GL = GL;
})(this);