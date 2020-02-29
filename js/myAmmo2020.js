var mylib2020 = mylib2020 || {};

mylib2020.initAmmo = function (gravity) {
    const collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
    const dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
    const overlappingPairCache = new Ammo.btDbvtBroadphase();
    const solver = new Ammo.btSequentialImpulseConstraintSolver();
    const world = new Ammo.btDiscreteDynamicsWorld(
        dispatcher,
        overlappingPairCache,
        solver,
        collisionConfiguration
    );
    const gv = new Ammo.btVector3(gravity.x, gravity.y, gravity.z);
    world.setGravity(gv);
    Ammo.destroy(gv);
    return [world, dispatcher, overlappingPairCache, solver, collisionConfiguration];
}

mylib2020.AmmoCollisionBuilder = class {
    constructor() {
        this.position = new Ammo.btVector3();
        this.rotation = new Ammo.btQuaternion();
        this.transform = new Ammo.btTransform();
        this.localInertia = new Ammo.btVector3();
        this.size = new Ammo.btVector3();
        this.scale = new Ammo.btVector3();
        this.factor = new Ammo.btVector3();
        this.vertex = new Ammo.btVector3();
        this.minLength = 0.01;
        this.collisionShapeMaterial = new THREE.MeshBasicMaterial({ color: 0x00ff00, wireframe: true });
    }

    addConvexHullShape(threeObject, vertices, scale) {
        const shape = new Ammo.btConvexHullShape();
        for (let v of vertices) {
            this.vertex.setValue(v.x, v.y, v.z);
            shape.addPoint(this.vertex);
        }
        this.scale.setValue(scale.x, scale.y, scale.z);
        shape.setLocalScaling(this.scale);
        mylib2020.addAmmoShape(threeObject, shape);
        return shape;
    }

    addPlaneShape(threeObject) {
        const geometryParams = threeObject.geometry.parameters;
        const w2 = geometryParams.width / 2; // X
        const h2 = geometryParams.height / 2; // Y
        const vertices = [
            new THREE.Vector3(-w2, -h2, 0),
            new THREE.Vector3(w2, -h2, 0),
            new THREE.Vector3(w2, h2, 0),
            new THREE.Vector3(-w2, h2, 0)
        ];
        return this.addConvexHullShape(threeObject, vertices, threeObject.scale);
    }

    addBoxShape(threeObject) {
        threeObject.geometry.computeBoundingBox();
        const bbox = threeObject.geometry.boundingBox;
        const x = Math.max(this.minLength, Math.abs(bbox.max.x - bbox.min.x));
        const y = Math.max(this.minLength, Math.abs(bbox.max.y - bbox.min.y));
        const z = Math.max(this.minLength, Math.abs(bbox.max.z - bbox.min.z));
        this.size.setValue(x * 0.5, y * 0.5, z * 0.5);
        this.scale.setValue(threeObject.scale.x, threeObject.scale.y, threeObject.scale.z);
        const shape = new Ammo.btBoxShape(this.size);
        shape.setLocalScaling(this.scale);
        mylib2020.addAmmoShape(threeObject, shape);
        return shape;
    }

    addSphereShape(threeObject) {
        threeObject.geometry.computeBoundingSphere();
        const sphere = threeObject.geometry.boundingSphere;
        this.position.setValue(0, 0, 0);
        this.scale.setValue(threeObject.scale.x, threeObject.scale.y, threeObject.scale.z);
        const shape = new Ammo.btMultiSphereShape([this.position], [sphere.radius], 1);
        shape.setLocalScaling(this.scale);
        mylib2020.addAmmoShape(threeObject, shape);
        return shape;
    }

    addCylinderShape(threeObject) {
        threeObject.geometry.computeBoundingBox();
        const bbox = threeObject.geometry.boundingBox;
        const x = Math.max(this.minLength, Math.abs(bbox.max.x - bbox.min.x));
        const y = Math.max(this.minLength, Math.abs(bbox.max.y - bbox.min.y)); // Height
        const z = Math.max(this.minLength, Math.abs(bbox.max.z - bbox.min.z));
        const radius = Math.max(x, z) / 2;
        this.size.setValue(radius, y * 0.5, radius);
        this.scale.setValue(threeObject.scale.x, threeObject.scale.y, threeObject.scale.z);
        const shape = new Ammo.btCylinderShape(this.size);
        shape.setLocalScaling(this.scale);
        mylib2020.addAmmoShape(threeObject, shape);
        return shape;
    }

    addConeShape(threeObject) {
        const geometryParams = threeObject.geometry.parameters;
        const radius = geometryParams.radius;
        const height = geometryParams.height;
        const shape = new Ammo.btConeShape(radius, height);
        this.scale.setValue(threeObject.scale.x, threeObject.scale.y, threeObject.scale.z);
        shape.setLocalScaling(this.scale);
        mylib2020.addAmmoShape(threeObject, shape);
        return shape;
    }

    calcBoundingBox(threeObject, scale, offset) {
        const bbox = new THREE.Box3();
        const center = new THREE.Vector3();
        bbox.setFromObject(threeObject);
        bbox.getCenter(center);
        threeObject.worldToLocal(center);
        const x = Math.max(this.minLength, Math.abs(bbox.max.x - bbox.min.x)) * scale.x;
        const y = Math.max(this.minLength, Math.abs(bbox.max.y - bbox.min.y)) * scale.y;
        const z = Math.max(this.minLength, Math.abs(bbox.max.z - bbox.min.z)) * scale.z;
        center.add(offset);
        return { size: { x, y, z }, center: center };
    }

    addBoundingCylinderShape(threeObject, params = null) {
        params = params || {};
        const showCollision = ('showCollision' in params) ? params.showCollision : false;
        const axis = ('axis' in params) ? params.axis.toLowerCase() : "y";
        if (axis != "x" && axis != "y" && axis != "z") {
            axis = "y";
        }
        const scale = ('scale' in params) ? params.scale : new THREE.Vector3(1, 1, 1);
        const offset = ('offset' in params) ? params.offset : new THREE.Vector3(0, 0, 0);

        let radius = 0;
        let height = 0;
        let rotation = new THREE.Vector3();
        const bbox = this.calcBoundingBox(threeObject, scale, offset);
        if (axis == "x") {
            height = bbox.size.x;
            radius = Math.max(bbox.size.y, bbox.size.z) / 2;
            rotation.z = THREE.Math.degToRad(90);
        } else if (axis == "y") {
            height = bbox.size.y;
            radius = Math.max(bbox.size.x, bbox.size.z) / 2;
        } else if (axis == "z") {
            height = bbox.size.z;
            radius = Math.max(bbox.size.x, bbox.size.y) / 2;
            rotation.x = THREE.Math.degToRad(90);
        }

        const geom = new THREE.CylinderGeometry(radius, radius, height, 8);
        const childMesh = new THREE.Mesh(geom, this.collisionShapeMaterial.clone());
        if (showCollision == false) {
            childMesh.material.transparent = true;
            childMesh.material.opacity = 0.0;
        }
        childMesh.name = threeObject.name + "_BCYLINDER";
        childMesh.rotation.set(rotation.x, rotation.y, rotation.z);
        childMesh.position.set(bbox.center.x, bbox.center.y, bbox.center.z);
        this.addCylinderShape(childMesh);
        threeObject.add(childMesh);
        return this.addCompoundShape(threeObject);
    }

    addBoundingSphereShape(threeObject, params = null) {
        params = params || {};
        const showCollision = ('showCollision' in params) ? params.showCollision : false;
        const scale = ('scale' in params) ? params.scale : new THREE.Vector3(1, 1, 1);
        const offset = ('offset' in params) ? params.offset : new THREE.Vector3(0, 0, 0);

        const bbox = this.calcBoundingBox(threeObject, scale, offset);
        const geom = new THREE.SphereGeometry(0.5, 8, 8);
        const childMesh = new THREE.Mesh(geom, this.collisionShapeMaterial.clone());
        if (showCollision == false) {
            childMesh.material.transparent = true;
            childMesh.material.opacity = 0.0;
        }
        childMesh.scale.set(bbox.size.x, bbox.size.y, bbox.size.z);
        childMesh.name = threeObject.name + "_BSPHERE";
        childMesh.position.set(bbox.center.x, bbox.center.y, bbox.center.z);
        this.addSphereShape(childMesh);
        threeObject.add(childMesh);
        return this.addCompoundShape(threeObject);
    }

    addBoundingBoxShape(threeObject, params = null) {
        params = params || {};
        const showCollision = ('showCollision' in params) ? params.showCollision : false;
        const scale = ('scale' in params) ? params.scale : new THREE.Vector3(1, 1, 1);
        const offset = ('offset' in params) ? params.offset : new THREE.Vector3(0, 0, 0);

        const bbox = this.calcBoundingBox(threeObject, scale, offset);
        const geom = new THREE.BoxGeometry(bbox.size.x, bbox.size.y, bbox.size.z);
        const childMesh = new THREE.Mesh(geom, this.collisionShapeMaterial.clone());
        if (showCollision == false) {
            childMesh.material.transparent = true;
            childMesh.material.opacity = 0.0;
        }
        childMesh.name = threeObject.name + "_BBOX";
        childMesh.position.set(bbox.center.x, bbox.center.y, bbox.center.z);
        this.addBoxShape(childMesh);
        threeObject.add(childMesh);
        return this.addCompoundShape(threeObject);
    }

    addCompoundShape(threeObject) {
        const shape = new Ammo.btCompoundShape();
        if (threeObject.userData.collisionShape) {
            threeObject.userData.collisionShapeOrg = threeObject.userData.collisionShape;
            this.transform.setIdentity();
            shape.addChildShape(this.transform, threeObject.userData.collisionShapeOrg);
        }
        threeObject.userData.collisionShape = shape;
        for (let child of threeObject.children) {
            const childShape = this.addCompoundShape(child);
            this.position.setValue(child.position.x, child.position.y, child.position.z);
            this.rotation.setValue(child.quaternion.x, child.quaternion.y, child.quaternion.z, child.quaternion.w);
            this.transform.setIdentity();
            this.transform.setOrigin(this.position);
            this.transform.setRotation(this.rotation);
            shape.addChildShape(this.transform, childShape);
        }
        return shape;
    }

    addRigidBody(threeObject, params = null) {
        if (threeObject.userData.collisionShape) {
            const rigidBody = this.rigidBody(threeObject.position, threeObject.quaternion, threeObject.userData.collisionShape, params);
            mylib2020.addAmmoRigidBody(threeObject, rigidBody);
            this.checkMovable(threeObject, params);
        }
    }

    checkMovable(threeObject, params = null) {
        params = params || {};
        threeObject.userData.movable = ('movable' in params) ? params.movable : false;
        if (threeObject.userData.movable) {
            threeObject.userData.linearVelocity = 0;
            threeObject.userData.angularVelocity = 0;
        }
    }

    rigidBody(pos, quat, shape, params = null) {
        params = params || {};
        const mass = ('mass' in params) ? params.mass : 1;
        const friction = ('friction' in params) ? params.friction : 0.5;
        const rollingFriction = ('rollingFriction' in params) ? params.friction : 0.1;
        const restitution = ('restitution' in params) ? params.restitution : 0;
        const kinematic = ('kinematic' in params) ? params.kinematic : false;
        const freezeRotationX = ('freezeRotationX' in params) ? params.freezeRotationX : false;
        const freezeRotationY = ('freezeRotationY' in params) ? params.freezeRotationY : false;
        const freezeRotationZ = ('freezeRotationZ' in params) ? params.freezeRotationZ : false;
        const linearDamping = ('linearDamping' in params) ? params.linearDamping : 0;
        const angularDamping = ('angularDamping' in params) ? params.angularDamping : 0;

        const margin = ('margin' in params) ? params.margin : 0.01;
        shape.setMargin(margin);
        this.localInertia.setValue(0, 0, 0);
        shape.calculateLocalInertia(mass, this.localInertia);

        this.position.setValue(pos.x, pos.y, pos.z);
        this.rotation.setValue(quat.x, quat.y, quat.z, quat.w);
        this.transform.setIdentity();
        this.transform.setOrigin(this.position);
        this.transform.setRotation(this.rotation);
        const motionState = new Ammo.btDefaultMotionState(this.transform);
        const rbInfo = new Ammo.btRigidBodyConstructionInfo(mass, motionState, shape, this.localInertia);
        const rigidBody = new Ammo.btRigidBody(rbInfo);
        rigidBody.setRollingFriction(rollingFriction);
        rigidBody.setFriction(friction);
        rigidBody.setRestitution(restitution);
        rigidBody.setDamping(linearDamping, angularDamping);
        if (kinematic) {
            rigidBody.setActivationState(Ammo.DISABLE_DEACTIVATION);
            rigidBody.setCollisionFlags(rigidBody.getCollisionFlags() | Ammo.CF_KINEMATIC_OBJECT);
        }
        if (freezeRotationX || freezeRotationY || freezeRotationZ) {
            this.factor.setValue(1, 1, 1);
            if (freezeRotationX) {
                this.factor.setX(0);
            }
            if (freezeRotationY) {
                this.factor.setY(0);
            }
            if (freezeRotationZ) {
                this.factor.setZ(0);
            }
            rigidBody.setAngularFactor(this.factor);
        }
        Ammo.destroy(rbInfo);
        return rigidBody;
    }

    destroy() {
        Ammo.destroy(this.position);
        Ammo.destroy(this.rotation);
        Ammo.destroy(this.transform);
        Ammo.destroy(this.localInertia);
        Ammo.destroy(this.size);
        Ammo.destroy(this.scale);
        Ammo.destroy(this.factor);
        Ammo.destroy(this.vertex);
    }
}

mylib2020.addAmmoRigidBody = function (threeObject, rigidBody) {
    mylib2020.removeAmmoRigidBody(threeObject);
    threeObject.userData.rigidBody = rigidBody;
    rigidBody.threeObject = threeObject;
}

mylib2020.removeAmmoRigidBody = function (threeObject) {
    if (threeObject.userData.rigidBody) {
        if (threeObject.userData.rigidBody.threeObject) {
            delete threeObject.userData.rigidBody.threeObject;
        }
        Ammo.destroy(threeObject.userData.rigidBody.getMotionState());
        Ammo.destroy(threeObject.userData.rigidBody);
        delete threeObject.userData.rigidBody;
    }
}

mylib2020.addAmmoShape = function (threeObject, shape) {
    mylib2020.removeAmmoShape(threeObject);
    threeObject.userData.collisionShape = shape;
}

mylib2020.removeAmmoShapeRecursive = function (threeObject) {
    mylib2020.removeAmmoShape(threeObject);
    for (let child of threeObject.children) {
        mylib2020.removeAmmoShapeRecursive(child);
    }
}

mylib2020.removeAmmoShape = function (threeObject) {
    if (threeObject.userData.collisionShape) {
        Ammo.destroy(threeObject.userData.collisionShape);
        delete threeObject.userData.collisionShape;
    }
    if (threeObject.userData.collisionShapeOrg) {
        Ammo.destroy(threeObject.userData.collisionShapeOrg);
        delete threeObject.userData.collisionShapeOrg;
    }
}

mylib2020.addImpulse = function (rigidBody, impulse = new THREE.Vector3(0, 0, 0)) {
    const imp = new Ammo.btVector3(impulse.x, impulse.y, impulse.z);
    rigidBody.applyCentralImpulse(imp);
    Ammo.destroy(imp);
}

mylib2020.addForce = function (rigidBody, force = new THREE.Vector3(0, 0, 0)) {
    const f = new Ammo.btVector3(force.x, force.y, force.z);
    rigidBody.applyCentralForce(f);
    Ammo.destroy(f);
}

mylib2020.applyAmmo = function (threeObject, rigidBody, btTransformBuffer) {
    rigidBody.getMotionState().getWorldTransform(btTransformBuffer);
    const p = btTransformBuffer.getOrigin();
    const q = btTransformBuffer.getRotation();
    threeObject.position.set(p.x(), p.y(), p.z());
    threeObject.quaternion.set(q.x(), q.y(), q.z(), q.w());
}

mylib2020.AmmoRigidBodyPose = class {
    constructor() {
        this.position = new Ammo.btVector3();
        this.quaternion = new Ammo.btQuaternion();
        this.transform = new Ammo.btTransform();
        this.ACTIVE_TAG = 1;
    }

    set(rigidBody, position, quaternion) {
        this.position.setValue(position.x, position.y, position.z);
        this.quaternion.setValue(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        this.transform.setIdentity();
        this.transform.setOrigin(this.position);
        this.transform.setRotation(this.quaternion);
        const motionState = new Ammo.btDefaultMotionState(this.transform);
        Ammo.destroy(rigidBody.getMotionState());
        rigidBody.forceActivationState(this.ACTIVE_TAG);
        rigidBody.activate();
        rigidBody.setMotionState(motionState);
    }

    synch(threeObject) {
        if (threeObject.userData.rigidBody) {
            this.set(threeObject.userData.rigidBody, threeObject.position, threeObject.quaternion);
        }
    }

    destroy() {
        Ammo.destroy(this.position);
        Ammo.destroy(this.quaternion);
        Ammo.destroy(this.transform);
    }
}

mylib2020.AmmoCollisionManager = class {
    constructor() {
        this.newCollisions = new Map();
        this.all = new Map();
        this.enter = new Map();
        this.stay = new Map();
        this.exit = new Map();
    }

    getAll() {
        return this.all.keys();
    }

    getEnter() {
        return this.enter.keys();
    }

    getExit() {
        return this.exit.keys();
    }

    getStay() {
        return this.stay.keys();
    }

    collisionData() {
        return { sourceContactPoints: [], targetContactPoints: [] };
    }

    addNewCollision(targetThreeObject) {
        if (!this.newCollisions.has(targetThreeObject)) {
            this.newCollisions.set(targetThreeObject, this.collisionData());
        }
    }

    update() {
        this.enter.clear();
        this.stay.clear();
        for (let k of this.newCollisions.keys()) {
            if (this.all.has(k)) {
                this.stay.set(k, this.newCollisions[k]);
                this.all.delete(k);
            } else {
                this.enter.set(k, this.newCollisions[k]);
            }
        }
        this.exit = this.all;
        this.all = this.newCollisions;
        this.newCollisions = new Map();
    }
}
mylib2020.AmmoManager = class {
    constructor(scene, gravity = { x: 0, y: -9.8, z: 0 }, subStep = 1) {
        this.scene = scene;
        [this.world, this.dispatcher, this.overlappingPairCache, this.solver, this.collisionConfiguration]
            = mylib2020.initAmmo(gravity);
        this.targetObjects = new Set();
        this.subStep = subStep;
        this.collisionBuilder = new mylib2020.AmmoCollisionBuilder();
        this.rigidBodyPose = new mylib2020.AmmoRigidBodyPose();
        this.transform = new Ammo.btTransform();
    }

    applyPhysics(threeObject) {
        if (threeObject.userData.rigidBody) {
            mylib2020.applyAmmo(threeObject, threeObject.userData.rigidBody, this.transform);
        }
    }

    addImpulse(threeObject, impulse = new THREE.Vector3(0, 0, 0)) {
        if (threeObject.userData.rigidBody) {
            mylib2020.addImpulse(threeObject.userData.rigidBody, impulse);
        }
    }

    addForce(threeObject, force = new THREE.Vector3(0, 0, 0)) {
        if (threeObject.userData.rigidBody) {
            mylib2020.addForce(threeObject.userData.rigidBody, force);
        }
    }

    registerObject(threeObject) {
        if (this.targetObjects.has(threeObject)) {
            return false;
        }
        this.scene.add(threeObject);
        this.targetObjects.add(threeObject);
        if (threeObject.userData.rigidBody) {
            this.world.addRigidBody(threeObject.userData.rigidBody);
            threeObject.userData.collision = new mylib2020.AmmoCollisionManager();
        }
    }

    has(threeObject) {
        return this.targetObjects.has(threeObject);
    }

    update(delta) {
        for (let k of this.targetObjects) {
            if (!k.userData.movable) {
                continue;
            }
            if (k.userData.linearVelocity) {
                k.translateOnAxis(mylib2020.FORWARD, k.userData.linearVelocity * delta);
            }
            if (k.userData.angularVelocity) {
                k.rotateY(k.userData.angularVelocity * delta);
            }
            this.rigidBodyPose.synch(k);
        }
        this.world.stepSimulation(delta, this.subStep);
        for (let k of this.targetObjects) {
            this.applyPhysics(k);
        }
        this.updateCollision();
    }

    updateCollision() {
        const numManifolds = this.dispatcher.getNumManifolds();
        for (let i = 0; i < numManifolds; i++) {
            const manifold = this.dispatcher.getManifoldByIndexInternal(i); // btPersistentManifold
            const obA = Ammo.btRigidBody.prototype.upcast(manifold.getBody0()); // btCollisionObject 
            const obB = Ammo.btRigidBody.prototype.upcast(manifold.getBody1()); // btCollisionObject
            if (!obA.threeObject || !obB.threeObject) {
                console.log("updateCollision: found unregistered objects");
                continue;
            }
            const numContacts = manifold.getNumContacts(); // オブジェクト間の衝突点数
            for (let j = 0; j < numContacts; j++) {
                const pt = manifold.getContactPoint(j);//btManifoldPoint
                if (pt.getDistance() <= 0.0) {
                    const ptA = pt.getPositionWorldOnA(); // btVector3
                    const ptB = pt.getPositionWorldOnB(); // btVector3
                }
            }
            if (numContacts > 0) {
                obA.threeObject.userData.collision.addNewCollision(obB.threeObject);
                obB.threeObject.userData.collision.addNewCollision(obA.threeObject);
            }
        }
        for (let obj of this.targetObjects) {
            obj.userData.collision.update();
        }
    }

    removePhysics(threeObject) {
        if (threeObject.userData.collision) {
            delete threeObject.userData.collision;
        }
        if (threeObject.userData.rigidBody) {
            this.world.removeRigidBody(threeObject.userData.rigidBody);
            mylib2020.removeAmmoRigidBody(threeObject);
            mylib2020.removeAmmoShapeRecursive(threeObject);
        }
    }

    remove(threeObject) {
        this.removePhysics(threeObject);
        this.targetObjects.delete(threeObject);
        this.scene.remove(threeObject);
    }

    clear() {
        for (let obj of this.targetObjects) {
            this.removePhysicsRecursive(obj);
            this.scene.remove(threeObject);
        }
        this.targetObjects.clear();
    }

    destroy() {
        this.clear();
        this.collisionBuilder.destroy();
        this.rigidBodyPose.destroy();
        Ammo.destroy(this.transform);
        Ammo.destroy(this.world);
        Ammo.destroy(this.dispatcher);
        Ammo.destroy(this.overlappingPairCache);
        Ammo.destroy(this.solver);
        Ammo.destroy(this.collisionConfiguration);
    }
}
