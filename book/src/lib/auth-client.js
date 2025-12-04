import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: "https://physical-ai-hackathon.vercel.app", // Auth Server URL
});
