const express = require('express');
const mongoose = require('mongoose');
const cors = require('cors');
const authRoutes = require('./routes/authRoutes');
require('dotenv').config();

const app = express();

// ========== Middleware ==========
app.use(cors());
app.use(express.json());

// ========== MongoDB Connection ==========
const mongoURI = process.env.MONGO_URI;

if (!mongoURI) {
  console.error("❌ MONGO_URI is not defined in .env file.");
  process.exit(1); // Stop the app
}

mongoose.connect(mongoURI, {
  useNewUrlParser: true,
  useUnifiedTopology: true,
})
  .then(() => console.log("✅ MongoDB connected successfully"))
  .catch((err) => {
    console.error("❌ MongoDB connection error:", err.message);
    process.exit(1);
  });

// ========== Routes ==========
app.use('/api/auth', authRoutes);

// ========== Default Route ==========
app.get('/', (req, res) => {
  res.send('🍇 Fruit Pilot Auth Backend Running...');
});

// ========== Start Server ==========
const PORT = process.env.PORT || 5000;
app.listen(PORT, () => {
  console.log(`🚀 Server running at: http://localhost:${PORT}`);
});
