var mylib2020 = mylib2020 || {};

mylib2020.initOimoPhysics = function () {
    const world = new OIMO.World()
    world.gravity = new OIMO.Vec3(0, -9.80665, 0)
    return world;
}

mylib2020.addOPhysRigidBody = function (threeObject, type, geometry, mass, friction, restitution, oimoPhysicsWorld) { //OIMO.RigidBodyType.STATIC, OIMO.RigidBodyType.DYNAMIC, OIMO.RigidBodyType.KINEMATIC
    const rigidConf = new OIMO.RigidBodyConfig();
    rigidConf.type = type;
    rigidBody = new OIMO.RigidBody(rigidConf);
    const shapeConf = new OIMO.ShapeConfig();
    shapeConf.geometry = geometry;
    shapeConf.friction = friction;
    shapeConf.restitution = restitution;
    rigidBody.addShape(new OIMO.Shape(shapeConf));
    const massData = new OIMO.MassData();
    massData.mass = mass;
    rigidBody.setMassData(massData);
    threeObject.userData.rigidBody = rigidBody;
    oimoPhysicsWorld.addRigidBody(threeObject.userData.rigidBody);
    return rigidBody;
}

mylib2020.addOPhysBox = function (threeObject, type, bbox, mass, friction, restitution, oimoPhysicsWorld, minLength = 0.01) {
    const size = {
        w: Math.max(minLength, Math.abs(bbox.max.x - bbox.min.x)),
        h: Math.max(minLength, Math.abs(bbox.max.y - bbox.min.y)),
        d: Math.max(minLength, Math.abs(bbox.max.z - bbox.min.z))
    };
    const geometry = new OIMO.BoxGeometry(new OIMO.Vec3(size.w / 2, size.h / 2, size.d / 2));
    return mylib2020.addOPhysRigidBody(threeObject, type, geometry, mass, friction, restitution, oimoPhysicsWorld);
}

mylib2020.addOPhysSphere = function (threeObject, type, radius, mass, friction, restitution, oimoPhysicsWorld) {
    const geometry = new OIMO.SphereGeometry(radius);
    return mylib2020.addOPhysRigidBody(threeObject, type, geometry, mass, friction, restitution, oimoPhysicsWorld);
}

mylib2020.applyOPhys = function (threeObject) {
    if (!threeObject.userData.rigidBody) {
        return false;
    }
    const rigidBody = threeObject.userData.rigidBody;
    const pos = rigidBody.getPosition()
    threeObject.position.set(pos.x, pos.y, pos.z);
    const ori = rigidBody.getOrientation();
    threeObject.quaternion.set(ori.x, ori.y, ori.z, ori.w);
    return true;
}

mylib2020.setOPhysRBodyState = function (threeObject) {
    if (!threeObject.userData.rigidBody) {
        return false;
    }
    const rigidBody = threeObject.userData.rigidBody;
    const p = threeObject.position;
    const q = threeObject.quaternion;
    rigidBody.setPosition(new OIMO.Vec3(p.x, p.y, p.z));
    rigidBody.setOrientation(new OIMO.Quat(q.x, q.y, q.z, q.w));
    return false;
}

mylib2020.OimoPhysicsManager = class {
    constructor() {
        this.world = mylib2020.initOimoPhysics();
        this.targetObjects = new Set();
    }

    addPlane(threeObject, type, mass = 0, friction = 0.5, restitution = 0.5, minLength = 0.01) {
        this.addBox(threeObject, type, mass, friction, restitution, minLength);
    }

    addBox(threeObject, type, mass = 1, friction = 0.5, restitution = 0.5, minLength = 0.01) {
        threeObject.geometry.computeBoundingBox();
        mylib2020.addOPhysBox(threeObject, type, threeObject.geometry.boundingBox, mass, friction, restitution, this.world, minLength);
        this.setRigidBodyState(threeObject);
        this.targetObjects.add(threeObject);
    }

    addSphere(threeObject, type, mass = 1, friction = 0.5, restitution = 0.5) {
        threeObject.geometry.computeBoundingSphere();
        mylib2020.addOPhysSphere(threeObject, type, threeObject.geometry.boundingSphere.radius, mass, friction, restitution, this.world);
        this.setRigidBodyState(threeObject);
        this.targetObjects.add(threeObject);
    }

    setRigidBodyState(threeObject) {
        mylib2020.setOPhysRBodyState(threeObject);
    }

    update(delta) {
        this.world.step(delta);
        for (let o of this.targetObjects) {
            mylib2020.applyOPhys(o);
        }
    }

    remove(threeObject) {
        delete this.targetObjects.delete(threeObject);
    }
}
