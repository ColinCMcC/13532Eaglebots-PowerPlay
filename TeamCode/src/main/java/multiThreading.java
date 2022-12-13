public class multiThreading extends Thread
{
        public void run(){
            System.out.println("task one");
        }
        public static void main(String args[]){
            multiThreading t1=new multiThreading();
            multiThreading t2=new multiThreading();
            multiThreading t3=new multiThreading();

            t1.start();
            t2.start();
            t3.start();
        }
    }
