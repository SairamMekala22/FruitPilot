import { Request, Response, NextFunction } from "express";
import jwt, { JwtPayload } from "jsonwebtoken";

// Extend Request to include userId
export interface AuthRequest extends Request {
  userId?: string;
}

export const authenticateToken = (
  req: AuthRequest,
  res: Response,
  next: NextFunction
) => {
  const authHeader = req.headers.authorization;
  const token = authHeader?.startsWith("Bearer ")
    ? authHeader.split(" ")[1]
    : null;

  if (!token) {
    return res.status(401).json({ error: "Access denied. No token provided." });
  }

  try {
    const secret = process.env.JWT_SECRET;
    if (!secret) throw new Error("JWT_SECRET is not defined");

    const decoded = jwt.verify(token, secret) as JwtPayload & { userId: string };
    req.userId = decoded.userId;
    next();
  } catch (err) {
    console.error("Token verification failed:", err);
    res.status(403).json({ error: "Invalid or expired token." });
  }
};
