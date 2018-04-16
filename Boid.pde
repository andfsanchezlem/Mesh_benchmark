class Boid {
  PShape shape;
  Node node;
  int grabsMouseColor;
  int avatarColor;
  ArrayList<PVector> vertices;
  ArrayList<Face> lista_caras;
  // fields
  Vector position, velocity, acceleration, alignment, cohesion, separation; // position, velocity, and acceleration in
  // a vector datatype
  float neighborhoodRadius; // radius in which it looks for fellow boids
  float maxSpeed = 4; // maximum magnitude for the velocity vector
  float maxSteerForce = .1f; // maximum magnitude of the steering vector
  float sc = 3; // scale factor for the render of the boid
  float flap = 0;
  float t = 0;

  Boid(Vector inPos) {
    grabsMouseColor = color(0, 0, 255);
    avatarColor = color(255, 255, 0);
    position = new Vector();
    position.set(inPos);
       
    node = new Node(scene) {
      // Note that within visit() geometry is defined at the
      // node local coordinate system.
      @Override
      public void visit() {
        if (animate)
          run(flock);
        render();
      }

      // Behaviour: tapping over a boid will select the node as
      // the eye reference and perform an eye interpolation to it.
      @Override
      public void interact(TapEvent event) {
        if (avatar != this && scene.eye().reference() != this) {
          avatar = this;
          scene.eye().setReference(this);
          scene.interpolateTo(this);
        }
      }
    };
    node.setPosition(new Vector(position.x(), position.y(), position.z()));
    velocity = new Vector(random(-1, 1), random(-1, 1), random(1, -1));
    acceleration = new Vector(0, 0, 0);
    neighborhoodRadius = 100;
    if(representation){
      VertexVertexR();
    }
    else{
      FaceVertexR();
    }
    
    
  }

  public void run(ArrayList<Boid> boids) {
    t += .1;
    flap = 10 * sin(t);
    // acceleration.add(steer(new Vector(mouseX,mouseY,300),true));
    // acceleration.add(new Vector(0,.05,0));
    if (avoidWalls) {
      acceleration.add(Vector.multiply(avoid(new Vector(position.x(), flockHeight, position.z())), 5));
      acceleration.add(Vector.multiply(avoid(new Vector(position.x(), 0, position.z())), 5));
      acceleration.add(Vector.multiply(avoid(new Vector(flockWidth, position.y(), position.z())), 5));
      acceleration.add(Vector.multiply(avoid(new Vector(0, position.y(), position.z())), 5));
      acceleration.add(Vector.multiply(avoid(new Vector(position.x(), position.y(), 0)), 5));
      acceleration.add(Vector.multiply(avoid(new Vector(position.x(), position.y(), flockDepth)), 5));
    }
    flock(boids);
    move();
    checkBounds();
  }

  Vector avoid(Vector target) {
    Vector steer = new Vector(); // creates vector for steering
    steer.set(Vector.subtract(position, target)); // steering vector points away from
    steer.multiply(1 / sq(Vector.distance(position, target)));
    return steer;
  }

  //-----------behaviors---------------

  void flock(ArrayList<Boid> boids) {
    //alignment
    alignment = new Vector(0, 0, 0);
    int alignmentCount = 0;
    //cohesion
    Vector posSum = new Vector();
    int cohesionCount = 0;
    //separation
    separation = new Vector(0, 0, 0);
    Vector repulse;
    for (int i = 0; i < boids.size(); i++) {
      Boid boid = boids.get(i);
      //alignment
      float distance = Vector.distance(position, boid.position);
      if (distance > 0 && distance <= neighborhoodRadius) {
        alignment.add(boid.velocity);
        alignmentCount++;
      }
      //cohesion
      float dist = dist(position.x(), position.y(), boid.position.x(), boid.position.y());
      if (dist > 0 && dist <= neighborhoodRadius) {
        posSum.add(boid.position);
        cohesionCount++;
      }
      //separation
      if (distance > 0 && distance <= neighborhoodRadius) {
        repulse = Vector.subtract(position, boid.position);
        repulse.normalize();
        repulse.divide(distance);
        separation.add(repulse);
      }
    }
    //alignment
    if (alignmentCount > 0) {
      alignment.divide((float) alignmentCount);
      alignment.limit(maxSteerForce);
    }
    //cohesion
    if (cohesionCount > 0)
      posSum.divide((float) cohesionCount);
    cohesion = Vector.subtract(posSum, position);
    cohesion.limit(maxSteerForce);

    acceleration.add(Vector.multiply(alignment, 1));
    acceleration.add(Vector.multiply(cohesion, 3));
    acceleration.add(Vector.multiply(separation, 1));
  }

  void move() {
    velocity.add(acceleration); // add acceleration to velocity
    velocity.limit(maxSpeed); // make sure the velocity vector magnitude does not
    // exceed maxSpeed
    position.add(velocity); // add velocity to position
    node.setPosition(position);
    node.setRotation(Quaternion.multiply(new Quaternion(new Vector(0, 1, 0), atan2(-velocity.z(), velocity.x())),
      new Quaternion(new Vector(0, 0, 1), asin(velocity.y() / velocity.magnitude()))));
    acceleration.multiply(0); // reset acceleration
  }

  void checkBounds() {
    if (position.x() > flockWidth)
      position.setX(0);
    if (position.x() < 0)
      position.setX(flockWidth);
    if (position.y() > flockHeight)
      position.setY(0);
    if (position.y() < 0)
      position.setY(flockHeight);
    if (position.z() > flockDepth)
      position.setZ(0);
    if (position.z() < 0)
      position.setZ(flockDepth);
  }
  void VertexVertexR(){
    
    vertices= new ArrayList<PVector>();
    
    PVector XZ1 = new PVector(3 * sc, 0, 2 * sc);
    PVector XZ2 = new PVector(3 * sc, 0, -2 * sc);
    PVector XZ3 = new PVector(-3 * sc, 0, 2 * sc);
    PVector XZ4 = new PVector(-3 * sc, 0, -2 * sc);
    PVector XY1 = new PVector(2 * sc, 3 * sc, 0);
    PVector XY2 = new PVector(-2 * sc, 3 * sc, 0);
    PVector XY3 = new PVector(2 * sc, -3 * sc, 0);
    PVector XY4 = new PVector(-2 * sc, -3 * sc, 0);
    PVector YZ1 = new PVector(0, 2 * sc, 3 * sc);
    PVector YZ2 = new PVector(0, -2 * sc, 3 * sc);
    PVector YZ3 = new PVector(0, 2 * sc, -3 * sc);
    PVector YZ4 = new PVector(0, -2 * sc, -3 * sc);

    vertices.add(XZ1);
    vertices.add(XZ2);
    vertices.add(XZ3);
    vertices.add(XZ4);
    vertices.add(XY1);
    vertices.add(XY2);
    vertices.add(XY3);
    vertices.add(XY4);
    vertices.add(YZ1);
    vertices.add(YZ2);
    vertices.add(YZ3);
    vertices.add(YZ4);  
    
    shape = createShape();
    shape.beginShape(TRIANGLE_STRIP);
    
    shape.vertex(vertices.get(0).x, vertices.get(0).y, vertices.get(0).z);
    shape.vertex(vertices.get(4).x, vertices.get(4).y, vertices.get(4).z);
    shape.vertex(vertices.get(8).x, vertices.get(8).y, vertices.get(8).z);
    shape.vertex(vertices.get(0).x, vertices.get(0).y, vertices.get(0).z);
    shape.vertex(vertices.get(9).x, vertices.get(9).y, vertices.get(9).z);
    shape.vertex(vertices.get(6).x, vertices.get(6).y, vertices.get(6).z);
    shape.vertex(vertices.get(0).x, vertices.get(0).y, vertices.get(0).z);
    shape.vertex(vertices.get(1).x, vertices.get(1).y, vertices.get(1).z);
    shape.vertex(vertices.get(4).x, vertices.get(4).y, vertices.get(4).z);
   
    shape.vertex(vertices.get(10).x, vertices.get(10).y, vertices.get(10).z);
    shape.vertex(vertices.get(1).x, vertices.get(1).y, vertices.get(1).z);
    shape.vertex(vertices.get(11).x, vertices.get(11).y, vertices.get(11).z);
    shape.vertex(vertices.get(6).x, vertices.get(6).y, vertices.get(6).z);
    shape.vertex(vertices.get(7).x, vertices.get(7).y, vertices.get(7).z);
    shape.vertex(vertices.get(9).x, vertices.get(9).y, vertices.get(9).z);
    shape.vertex(vertices.get(2).x, vertices.get(2).y, vertices.get(2).z);
    shape.vertex(vertices.get(8).x, vertices.get(8).y, vertices.get(8).z);
    shape.vertex(vertices.get(5).x, vertices.get(5).y, vertices.get(5).z);    
    shape.vertex(vertices.get(4).x, vertices.get(4).y, vertices.get(4).z);
    shape.vertex(vertices.get(10).x, vertices.get(10).y, vertices.get(10).z);
    shape.vertex(vertices.get(5).x, vertices.get(5).y, vertices.get(5).z);
    shape.vertex(vertices.get(3).x, vertices.get(3).y, vertices.get(3).z);
    shape.vertex(vertices.get(2).x, vertices.get(2).y, vertices.get(2).z);
    shape.vertex(vertices.get(7).x, vertices.get(7).y, vertices.get(7).z);
    shape.vertex(vertices.get(3).x, vertices.get(3).y, vertices.get(3).z);
    shape.vertex(vertices.get(11).x, vertices.get(11).y, vertices.get(11).z);
    shape.vertex(vertices.get(10).x, vertices.get(10).y, vertices.get(10).z);

    shape.endShape();
  }
  void VertexVertexI(int kind){
       vertices= new ArrayList<PVector>();
    
    PVector XZ1 = new PVector(3 * sc, 0, 2 * sc);
    PVector XZ2 = new PVector(3 * sc, 0, -2 * sc);
    PVector XZ3 = new PVector(-3 * sc, 0, 2 * sc);
    PVector XZ4 = new PVector(-3 * sc, 0, -2 * sc);
    PVector XY1 = new PVector(2 * sc, 3 * sc, 0);
    PVector XY2 = new PVector(-2 * sc, 3 * sc, 0);
    PVector XY3 = new PVector(2 * sc, -3 * sc, 0);
    PVector XY4 = new PVector(-2 * sc, -3 * sc, 0);
    PVector YZ1 = new PVector(0, 2 * sc, 3 * sc);
    PVector YZ2 = new PVector(0, -2 * sc, 3 * sc);
    PVector YZ3 = new PVector(0, 2 * sc, -3 * sc);
    PVector YZ4 = new PVector(0, -2 * sc, -3 * sc);

    vertices.add(XZ1);
    vertices.add(XZ2);
    vertices.add(XZ3);
    vertices.add(XZ4);
    vertices.add(XY1);
    vertices.add(XY2);
    vertices.add(XY3);
    vertices.add(XY4);
    vertices.add(YZ1);
    vertices.add(YZ2);
    vertices.add(YZ3);
    vertices.add(YZ4);  
     beginShape(kind);
    vertex(vertices.get(0).x, vertices.get(0).y, vertices.get(0).z);
    vertex(vertices.get(4).x, vertices.get(4).y, vertices.get(4).z);
    vertex(vertices.get(8).x, vertices.get(8).y, vertices.get(8).z);
    vertex(vertices.get(0).x, vertices.get(0).y, vertices.get(0).z);
    vertex(vertices.get(9).x, vertices.get(9).y, vertices.get(9).z);
    vertex(vertices.get(6).x, vertices.get(6).y, vertices.get(6).z);
    vertex(vertices.get(0).x, vertices.get(0).y, vertices.get(0).z);
    vertex(vertices.get(1).x, vertices.get(1).y, vertices.get(1).z);
    vertex(vertices.get(4).x, vertices.get(4).y, vertices.get(4).z);
    vertex(vertices.get(10).x, vertices.get(10).y, vertices.get(10).z);
    vertex(vertices.get(1).x, vertices.get(1).y, vertices.get(1).z);
    vertex(vertices.get(11).x, vertices.get(11).y, vertices.get(11).z);
    vertex(vertices.get(6).x, vertices.get(6).y, vertices.get(6).z);
    vertex(vertices.get(7).x, vertices.get(7).y, vertices.get(7).z);
    vertex(vertices.get(9).x, vertices.get(9).y, vertices.get(9).z);
    vertex(vertices.get(2).x, vertices.get(2).y, vertices.get(2).z);
    vertex(vertices.get(8).x, vertices.get(8).y, vertices.get(8).z);
    vertex(vertices.get(5).x, vertices.get(5).y, vertices.get(5).z);    
    vertex(vertices.get(4).x, vertices.get(4).y, vertices.get(4).z);
    vertex(vertices.get(10).x, vertices.get(10).y, vertices.get(10).z);
    vertex(vertices.get(5).x, vertices.get(5).y, vertices.get(5).z);
    vertex(vertices.get(3).x, vertices.get(3).y, vertices.get(3).z);
    vertex(vertices.get(2).x, vertices.get(2).y, vertices.get(2).z);
    vertex(vertices.get(7).x, vertices.get(7).y, vertices.get(7).z);
    vertex(vertices.get(3).x, vertices.get(3).y, vertices.get(3).z);
    vertex(vertices.get(11).x, vertices.get(11).y, vertices.get(11).z);
    vertex(vertices.get(10).x, vertices.get(10).y, vertices.get(10).z);
    endShape();
  }
  void FaceVertexR(){
    vertices= new ArrayList<PVector>();
    lista_caras = new  ArrayList<Face>();
   
    PVector XZ1 = new PVector(3 * sc, 0, 2 * sc);
    PVector XZ2 = new PVector(3 * sc, 0, -2 * sc);
    PVector XZ3 = new PVector(-3 * sc, 0, 2 * sc);
    PVector XZ4 = new PVector(-3 * sc, 0, -2 * sc);
    PVector XY1 = new PVector(2 * sc, 3 * sc, 0);
    PVector XY2 = new PVector(-2 * sc, 3 * sc, 0);
    PVector XY3 = new PVector(2 * sc, -3 * sc, 0);
    PVector XY4 = new PVector(-2 * sc, -3 * sc, 0);
    PVector YZ1 = new PVector(0, 2 * sc, 3 * sc);
    PVector YZ2 = new PVector(0, -2 * sc, 3 * sc);
    PVector YZ3 = new PVector(0, 2 * sc, -3 * sc);
    PVector YZ4 = new PVector(0, -2 * sc, -3 * sc);

    Face f1 = new Face(XZ1,XY1,YZ1);
    Face f2 = new Face(YZ1,XZ1,YZ2);
    Face f3 = new Face(XZ1,YZ2,XY3);
    Face f4 = new Face(XY3,XZ1,XZ2);
    Face f5 = new Face(XZ1,XZ2,XY1); 
    Face f6 = new Face(XY1,YZ3,XZ2);
    Face f7 = new Face(YZ3,XZ2,YZ4);
    Face f8 = new Face(XZ2,YZ4,XY3);
    Face f9 = new Face(YZ4,XY3,XY4);
    Face f10 = new Face(YZ2,XZ3,YZ1);
    //Face f11 = new Face();
    //Face f12 = new Face();
    //Face f13 = new Face();
    //Face f14 = new Face();
    //Face f15 = new Face();
    //Face f16 = new Face();
    //Face f17 = new Face();
    //Face f18 = new Face();
    //Face f19 = new Face();
    //Face f20 = new Face();
  
    vertices.add(XZ1);
    vertices.add(XZ2);
    vertices.add(XZ3);
    vertices.add(XZ4);
    vertices.add(XY1);
    vertices.add(XY2);
    vertices.add(XY3);
    vertices.add(XY4);
    vertices.add(YZ1);
    vertices.add(YZ2);
    vertices.add(YZ3);
    vertices.add(YZ4);  
    
    lista_caras.add(f1);
    lista_caras.add(f2);
    lista_caras.add(f3);
    lista_caras.add(f4);
    lista_caras.add(f5);
    lista_caras.add(f6);
    lista_caras.add(f7);
    lista_caras.add(f8);
    lista_caras.add(f9);
    lista_caras.add(f10);
    
    shape = createShape();
    shape.beginShape(TRIANGLE_STRIP);
    
    for(Face f: lista_caras){
      for(PVector v : f.vertex_list){
        shape.vertex(v.x, v.y, v.z);
      }
    }
      
    shape.endShape();
  }
  void FaceVertexI(int kind){
    vertices= new ArrayList<PVector>();
    lista_caras = new  ArrayList<Face>();
   
    PVector XZ1 = new PVector(3 * sc, 0, 2 * sc);
    PVector XZ2 = new PVector(3 * sc, 0, -2 * sc);
    PVector XZ3 = new PVector(-3 * sc, 0, 2 * sc);
    PVector XZ4 = new PVector(-3 * sc, 0, -2 * sc);
    PVector XY1 = new PVector(2 * sc, 3 * sc, 0);
    PVector XY2 = new PVector(-2 * sc, 3 * sc, 0);
    PVector XY3 = new PVector(2 * sc, -3 * sc, 0);
    PVector XY4 = new PVector(-2 * sc, -3 * sc, 0);
    PVector YZ1 = new PVector(0, 2 * sc, 3 * sc);
    PVector YZ2 = new PVector(0, -2 * sc, 3 * sc);
    PVector YZ3 = new PVector(0, 2 * sc, -3 * sc);
    PVector YZ4 = new PVector(0, -2 * sc, -3 * sc);

    Face f1 = new Face(XZ1,XY1,YZ1);
    Face f2 = new Face(YZ1,XZ1,YZ2);
    Face f3 = new Face(XZ1,YZ2,XY3);
    Face f4 = new Face(XY3,XZ1,XZ2);
    Face f5 = new Face(XZ1,XZ2,XY1); 
    Face f6 = new Face(XY1,YZ3,XZ2);
    Face f7 = new Face(YZ3,XZ2,YZ4);
    Face f8 = new Face(XZ2,YZ4,XY3);
    Face f9 = new Face(YZ4,XY3,XY4);
    Face f10 = new Face(YZ2,XZ3,YZ1);
    //Face f11 = new Face();
    //Face f12 = new Face();
    //Face f13 = new Face();
    //Face f14 = new Face();
    //Face f15 = new Face();
    //Face f16 = new Face();
    //Face f17 = new Face();
    //Face f18 = new Face();
    //Face f19 = new Face();
    //Face f20 = new Face();
  
    vertices.add(XZ1);
    vertices.add(XZ2);
    vertices.add(XZ3);
    vertices.add(XZ4);
    vertices.add(XY1);
    vertices.add(XY2);
    vertices.add(XY3);
    vertices.add(XY4);
    vertices.add(YZ1);
    vertices.add(YZ2);
    vertices.add(YZ3);
    vertices.add(YZ4);  
    
    lista_caras.add(f1);
    lista_caras.add(f2);
    lista_caras.add(f3);
    lista_caras.add(f4);
    lista_caras.add(f5);
    lista_caras.add(f6);
    lista_caras.add(f7);
    lista_caras.add(f8);
    lista_caras.add(f9);
    lista_caras.add(f10);
    beginShape(kind);
    for(Face f: lista_caras){
      for(PVector v : f.vertex_list){
        vertex(v.x, v.y, v.z);
      }
    }    
    endShape();
  }
  void render() {
    pushStyle();

    // uncomment to draw boid axes
    //scene.drawAxes(10);

    int kind = TRIANGLES;
    strokeWeight(2);
    stroke(color(0, 255, 0));
    fill(color(255, 0, 0, 125));

    // visual modes
    switch(mode) {
    case 1:
      noFill();
      break;
    case 2:
      noStroke();
      break;
    case 3:
      strokeWeight(3);
      kind = POINTS;
      break;
    }

    // highlight boids under the mouse
    if (node.track(mouseX, mouseY)) {
      noStroke();
      fill(grabsMouseColor);
    }

    // highlight avatar
    if (node == avatar) {
      noStroke();
      fill(avatarColor);
    }
    if(modo){
      if(representation){
        VertexVertexI(TRIANGLE_STRIP);
      }else{
        FaceVertexI(TRIANGLE_STRIP);
      }
  
      popStyle();
    }else{
      
      shape(shape);
         
    }
  }

}