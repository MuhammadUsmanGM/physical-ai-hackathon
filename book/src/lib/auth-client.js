import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  // baseURL: "http://localhost:4000", // Local Auth Server URL
  baseURL: "https://physical-ai-hackathon.vercel.app", // Production Auth Server URL
});
