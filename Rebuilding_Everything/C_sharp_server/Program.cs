using System;
using System.Net;
using System.Collections.Concurrent;
using System.Text;
using System.Collections.Generic;
using System.Net.Sockets;


namespace HelloWorld
{
    class Program
    {
        public static ConcurrentDictionary<string, int> shitty_sql = 
            new ConcurrentDictionary<string, int>(5, 12);

        static void Main(string[] args)
        {
            Console.WriteLine("Starting the server...\n(Press 'x' to terminate server)");
            bool keep_going = true;

            shitty_sql.AddOrUpdate("atTar", -1,(k,v) => -1);

            // Gest Host IP address that is used to establish a connection
            IPHostEntry host = Dns.GetHostEntry("localhost");
            IPAddress ipAddress = host.AddressList[0];
            IPEndPoint localEndPoint = new IPEndPoint(ipAddress, 508);

            Console.WriteLine($"Server started with IP {ipAddress} and endpoint {localEndPoint}.");

            try {
                // Create a socket that will use TCP protocol
                // Socket listener = new Socket(ipAddress.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
                TcpListener listener = new TcpListener(localEndPoint);
                // A socket must be associated with an endpoint using the Bind method
                // listener.Bind(localEndPoint);
                Console.WriteLine("Waiting for a connection...");
                while(keep_going) {
                    keep_going = (Console.ReadKey().KeyChar == 'x');

                    if (listener.Pending()) {
                        new Thread(() =>
                        {
                            TcpClient client = listener.AcceptTcpClient();
                            NetworkStream stream = client.GetStream();
                            client_thread(client, stream);
                        }).Start();
                    }
                }  
            } catch (Exception e) {
                Console.WriteLine(e.ToString());
            }

        }

        public static void client_thread(TcpClient client, NetworkStream stream) {
            int[] refresh_rate = new int[250];
            int i = 0;
            while(true){
                shitty_sql.
                i++;
            }

        }
    }
}