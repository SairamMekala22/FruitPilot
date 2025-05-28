import { createContext, useState, useEffect, ReactNode } from "react";
import {jwtDecode} from "jwt-decode";

// Define decoded token type (update if your JWT has more fields)
type DecodedToken = {
  exp: number; // expiration time in seconds
  username?: string; // if you include username in token
};

const token = localStorage.getItem("token"); // or sessionStorage.getItem()
if (token) {
  const decoded = jwtDecode(token);
  console.log(decoded); // use this to get user info or expiration
}


// Define context type
type AuthContextType = {
  token: string | null;
  isAuthenticated: boolean;
  login: (token: string) => void;
  logout: () => void;
};

// Create context with default values
export const AuthContext = createContext<AuthContextType>({
  token: null,
  isAuthenticated: false,
  login: () => {},
  logout: () => {},
});

// Props type for provider
interface AuthProviderProps {
  children: ReactNode;
}

// AuthProvider component
export const AuthProvider = ({ children }: AuthProviderProps) => {
  const [token, setToken] = useState<string | null>(null);
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  useEffect(() => {
    const storedToken = localStorage.getItem("token");
    if (storedToken) {
      try {
        const decoded: DecodedToken = jwtDecode(storedToken);
        const now = Date.now() / 1000; // current time in seconds

        if (decoded.exp > now) {
          setToken(storedToken);
          setIsAuthenticated(true);
        } else {
          // Token expired
          localStorage.removeItem("token");
          setToken(null);
          setIsAuthenticated(false);
        }
      } catch (err) {
        // Invalid token
        localStorage.removeItem("token");
        setToken(null);
        setIsAuthenticated(false);
      }
    }
  }, []);

  const login = (newToken: string) => {
    localStorage.setItem("token", newToken);
    setToken(newToken);
    setIsAuthenticated(true);
  };

  const logout = () => {
    localStorage.removeItem("token");
    setToken(null);
    setIsAuthenticated(false);
  };

  return (
    <AuthContext.Provider value={{ token, isAuthenticated, login, logout }}>
      {children}
    </AuthContext.Provider>
  );
};
