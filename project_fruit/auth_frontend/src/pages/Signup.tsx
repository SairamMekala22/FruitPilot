import { useState } from "react";
import { useNavigate, Link } from "react-router-dom"; // Import Link
import api from "../services/api";
import "./Signup.css";

const Signup = () => {
  const [username, setUsername] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const navigate = useNavigate();

  const handleSignup = async () => {
    try {
      await api.post("/auth/signup", { username, email, password });
      navigate("/login");
    } catch (err) {
      alert("Signup failed. Email may already exist.");
    }
  };

  return (
    <div className="signup-container">
      <h2 className="signup-header">Signup</h2>
      <input
        className="signup-input"
        placeholder="Username"
        onChange={e => setUsername(e.target.value)}
      />
      <input
        className="signup-input"
        placeholder="Email"
        onChange={e => setEmail(e.target.value)}
      />
      <input
        className="signup-input"
        placeholder="Password"
        type="password"
        onChange={e => setPassword(e.target.value)}
      />
      <button className="signup-button" onClick={handleSignup}>
        Signup
      </button>

      <p className="login-link">
        Already have an account? <Link to="/login">Log in here</Link>
      </p>
    </div>
  );
};

export default Signup;
