function VirtualButton(parentobj, options) {
	this.node = new THREE.Object3D();
	parentobj.add(this.node);

	this.options = options;
	this.createCollisionObject();

	this.node.position.copy(this.options.position);

	this.active = true;
	this.onClick = null;
	this.onView = null;
}


VirtualButton.prototype.createCollisionObject = function() {
	var geometry = new THREE.IcosahedronGeometry(this.options.radius, 2);
	var mat = new THREE.MeshBasicMaterial( { color: 0xff0000 } );
	this.collisionObj = new THREE.Mesh( geometry, mat );
	if(!this.options.debug) {
		this.collisionObj.visible = false;
	}

	this.node.add(this.collisionObj);
};

VirtualButton.prototype.addDisplayObject = function(obj) {
	this.dobj = obj;
	this.node.add(obj);
};

VirtualButton.prototype.getWorldPos = function() {
	var wp = new THREE.Vector3();
	wp.setFromMatrixPosition( this.node.matrixWorld );
	return wp;
};

VirtualButton.prototype.getCollider = function() {
	return this.collisionObj;
};


function VirtualButtonManager(manageropts) {
	this._options = {
		screenWidth: 800,
		screenHeight: 600
	};
	$.extend(this._options, manageropts);
	this._buttons = [];

	this._raycaster = new THREE.Raycaster();
}

VirtualButtonManager.prototype.addButton = function(button) {
	this._buttons.push(button);
};

VirtualButtonManager.prototype._computeClickRay = function(relX, relY) {
	if(!this._projector) {

	}
};

// apply a function f to objects along a ray specified by screen position
// (in normalized coordinates) [x,y]
// if closestOnly is set to true, only apply to the closest intersection,
// otherwise apply to every object
VirtualButtonManager.prototype._applyRay = function(x, y, f, closestOnly) {
	var nearest = 10000.0;
	var nearobj = null;

	this._raycaster.setFromCamera( new THREE.Vector2(x,y), this._cam );

	for(var i = 0; i < this._buttons.length; ++i) {
		var intersects = this._raycaster.intersectObject( this._buttons[i].getCollider(), false );
		if(intersects.length > 0) {
			if(!closestOnly) {
				f(this._buttons[i]);
			} else {
				if(intersects[0].distance < nearest) {
					nearest = intersects[0].distance;
					nearobj = this._buttons[i];
				}
			}
		}
	}

	if(closestOnly && (nearobj != null)) {
		f(nearobj);
	}
};

VirtualButtonManager.prototype.updateClick = function(clickX, clickY) {
	var relX = (clickX / this._options.screenWidth) * 2.0 - 1.0;
	var relY = -(clickY / this._options.screenHeight) * 2.0 - 1.0;

	this._applyRay(relX, relY, function(button) {
		if(button.onClick) {
			button.onClick();
		}
	}, true);
};

VirtualButtonManager.prototype.updateAll = function() {
	for(var i = 0; i < this._buttons.length; ++i) {
		if(this._buttons[i].onUpdate) {
			this._buttons[i].onUpdate();
		}
	}
};

VirtualButtonManager.prototype.updateView = function(cam) {
	this._cam = cam;

	this._applyRay(0.0, 0.0, function(button) {
		if(button.onView) {
			button.onView();
		}
	}, true);
};
