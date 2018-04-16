class Face{
  
  int id;
  PVector p1; 
  PVector p2;
  PVector p3;
  ArrayList<PVector> vertex_list;
  
  Face(PVector p1, PVector p2, PVector p3){
    this.p1 = p1;
    this.p2 = p2;
    this.p3 = p3;
    vertex_list= new ArrayList<PVector>();
    
    vertex_list.add(p1);
    vertex_list.add(p2);
    vertex_list.add(p3);
     
    
    
  }  

}