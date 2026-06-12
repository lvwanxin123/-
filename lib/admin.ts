const DEFAULT_ADMIN_EMAILS=["2543804778@qq.com"];
export function getAdminEmails(){const emails=process.env.ADMIN_EMAILS?.split(",").map((email)=>email.trim().toLowerCase()).filter(Boolean);return emails?.length?emails:DEFAULT_ADMIN_EMAILS;}
export function isAdminEmail(email?:string|null){return Boolean(email&&getAdminEmails().includes(email.toLowerCase()));}
