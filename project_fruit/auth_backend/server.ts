import express from "express";
import mongoose from "mongoose";
import dotenv from "dotenv";
import cors from "cors";
import { createServer } from "http";
import { Server } from "socket.io";
import authRoutes from "./routes/authRoutes";
import droneRoutes from "./routes/droneRoutes";
import { droneController } from "./controllers/droneController";

dotenv.config();
const app = express();
const httpServer = createServer(app);
const io = new Server(httpServer, {
  cors: {
    origin: "http://localhost:5173", // Vite's default port
    methods: ["GET", "POST"]
  }
});

app.use(cors());
app.use(express.json());
app.use("/api/auth", authRoutes);
app.use("/api/drone", droneRoutes);

// WebSocket connection handling
io.on("connection", (socket) => {
  console.log("Client connected");

  // Forward drone data to connected clients
  droneController.on("droneData", (data) => {
    socket.emit("droneData", data);
  });

  socket.on("disconnect", () => {
    console.log("Client disconnected");
  });
});

mongoose
  .connect(process.env.MONGO_URI || "")
  .then(() => {
    httpServer.listen(5000, () => console.log("Server running on port 5000"));
  })
  .catch((err) => console.error("MongoDB connection error:", err));
